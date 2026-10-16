// Example: an existing Arduino ESP32 project extended with XCP on UDP/IP (x1xcp kit).
// The lines marked "XCP" are what the extension needs; everything else is the existing project.

#include <Arduino.h>
#include <WiFi.h>
#include <x1xcp.h> // XCP

#include <cmath>

#ifndef WIFI_SSID // or as build flags (platformio.ini)
#define WIFI_SSID "my-network"
#define WIFI_PASSWORD "my-password"
#endif

// Global variables: measurable and calibratable (listed in a2l/variables.json). "volatile" keeps
// the compiler from holding them in registers, so that XCP sees and changes the real values.
enum Mode : std::uint8_t { MODE_OFF = 0, MODE_AUTO = 1, MODE_MANUAL = 2 };
volatile float setpoint = 50.0F;          // parameter
volatile Mode mode = MODE_AUTO;           // parameter (enum: texts in the A2L)
volatile std::uint32_t loop_counter = 0;  // measurement
volatile std::int16_t temperature_raw = 0;
volatile std::uint8_t relay_on = 0;
struct Status {
    std::uint32_t uptimeMs;
    std::int8_t rssi;
    std::uint8_t xcpConnected : 1;
};
volatile Status status = {};
volatile float samples[8] = {};

// XCP: DAQ events, the same order as "events" in a2l/variables.json
static const x1xcp::Event kEvents[] = {
    {"10ms", 10},          // triggered by controlTask (synchronous with the values)
    {"100ms", 100, true},  // triggered by the XCP task itself
};

static void controlTask(void*)
{
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
        const std::uint32_t now = static_cast<std::uint32_t>(wake) * portTICK_PERIOD_MS;
        loop_counter = loop_counter + 1;
        relay_on = (mode == MODE_AUTO && (now / 1000) % 2 == 0) ? 1 : 0;
        temperature_raw = static_cast<std::int16_t>(600 + 100 * std::sin(now * 1e-3F)); // 20 degC +- 10
        for (int i = 7; i > 0; --i) {
            samples[i] = samples[i - 1];
        }
        samples[0] = setpoint * std::sin(now * 1e-3F);
        x1xcp::trigger(0, now); // XCP: sample event "10ms" now, timestamp = this cycle
    }
}

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // XCP: modem sleep delays packets by up to 100 ms
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    x1xcp::Config xcp; // XCP
    xcp.events = kEvents;
    xcp.eventCount = sizeof(kEvents) / sizeof(kEvents[0]);
    if (!x1xcp::begin(xcp)) {
        Serial.println("XCP: UDP socket failed");
    }

    xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
}

void loop()
{
    status.uptimeMs = millis();
    status.rssi = static_cast<std::int8_t>(WiFi.RSSI());
    status.xcpConnected = x1xcp::stats().connected ? 1 : 0;
    static bool shown = false;
    if (!shown && WiFi.status() == WL_CONNECTED) {
        Serial.printf("WLAN connected: %s, XCP on UDP port 5555\n", WiFi.localIP().toString().c_str());
        shown = true;
    }
    delay(100);
}
