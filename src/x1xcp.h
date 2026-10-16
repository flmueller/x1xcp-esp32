#pragma once

// x1xcp: XCP on UDP/IP slave for ESP32 projects (XCP-Ray, REQ-FW-010). Measurement (dynamic DAQ)
// and calibration of global variables over WLAN or Ethernet; the A2L is generated from the ELF
// file with x1-elf2a2l (scripts/x1_a2l.py).
//
// Usage (Arduino or ESP-IDF, FreeRTOS tick 1 ms = CONFIG_FREERTOS_HZ 1000):
//
//   static const x1xcp::Event kEvents[] = {{"1ms", 1}, {"10ms", 10}, {"100ms", 100}};
//   x1xcp::Config config;
//   config.events = kEvents;
//   config.eventCount = 3;
//   x1xcp::begin(config);          // after the network interface exists (e.g. after WiFi.begin)
//   ...
//   x1xcp::trigger(0, nowMs);      // in the task that computes the values of event 0, after computing
//
// Events with autoTrigger are triggered by the XCP task itself (simpler, but not synchronous with
// the application). The events must match the "events" of the A2L variable list (same order).

#include <cstddef>
#include <cstdint>

namespace x1xcp {

struct Event {
    const char* name = "";       ///< as in the A2L
    std::uint16_t cycleMs = 0;   ///< nominal cycle (GET_DAQ_EVENT_INFO), 0 = not cyclic
    bool autoTrigger = false;    ///< triggered by the XCP task every cycleMs instead of by trigger()
};

struct Config {
    std::uint16_t port = 5555;     ///< UDP port of XCP on UDP/IP
    const Event* events = nullptr; ///< DAQ event channels 0..eventCount-1
    std::size_t eventCount = 0;
    /// DTOs are collected for this time per UDP datagram (fewer packets, exact timestamps). May
    /// point to a calibration variable; nullptr = 10 ms.
    const volatile std::uint8_t* flushMs = nullptr;
    int core = 1;          ///< core of the XCP task (the WLAN driver runs on core 0)
    unsigned priority = 0; ///< FreeRTOS priority of the XCP task, 0 = configMAX_PRIORITIES - 2
};

/// Opens the UDP port and starts the XCP task and the sender task. False on error (socket).
bool begin(const Config& config);

/// Event `event` occurred: samples the running DAQ lists of the event now, in the caller's task.
/// Thread-safe; takes a few microseconds plus the copy of the measured data. The timestamp is
/// the FreeRTOS tick in ms at the call.
void trigger(std::uint16_t event);
/// Same, with the timestamp in ms given by the application, e.g. the wake time of its cyclic
/// task: exactly one cycle apart even if the task runs a little late (no "missed cycles").
void trigger(std::uint16_t event, std::uint32_t timestampMs);

struct Stats {
    bool connected = false;     ///< a master is connected
    bool daqRunning = false;
    std::uint32_t commands = 0; ///< XCP commands processed
    std::uint32_t dtos = 0;     ///< DAQ packets queued
    std::uint32_t datagrams = 0;
    std::uint32_t overloads = 0;   ///< DAQ packets that did not fit (the master is told)
    std::uint32_t sendErrors = 0;  ///< datagrams that did not fit into the send queue
    std::uint32_t sendRetries = 0; ///< sendto() retries (WLAN busy)
};
Stats stats();

} // namespace x1xcp
