// x1xcp: XCP on UDP/IP for ESP32 (XCP-Ray, REQ-FW-010), see x1xcp.h.
//
// The XCP task (1 ms period) receives and answers commands and triggers the automatic events;
// trigger() samples in the caller's task. Both use the portable slave (x1/xcpslave) under one
// mutex. Packets get the XCP on UDP/IP header (LEN, CTR) and are collected into datagrams; a
// sender task on core 0 (where the WLAN driver runs) calls sendto(), which can block for more
// than a millisecond while the WLAN retransmits.

#include "x1xcp.h"

#include "x1/xcpslave/xcp_slave.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <soc/soc.h>

#include <fcntl.h>

#include <cstring>

namespace x1xcp {

namespace {

using x1::xcpslave::EventChannel;
using x1::xcpslave::IDtoTransmitter;
using x1::xcpslave::IMemory;
using x1::xcpslave::SlaveConfig;
using x1::xcpslave::XcpSlave;

constexpr std::size_t kMaxDatagram = 1400; // below the WLAN/Ethernet MTU
constexpr std::size_t kQueueDepth = 12;
constexpr std::size_t kMaxEvents = 16;

/// XCP addresses = CPU addresses. Writes only to internal data RAM, reads also from the
/// flash-mapped constant data (ranges of the chip from soc/soc.h).
class EspMemory final : public IMemory {
public:
    bool read(std::uint32_t address, std::uint8_t* out, std::size_t size) override
    {
        if (!inRange(address, size, SOC_DRAM_LOW, SOC_DRAM_HIGH) && !inRange(address, size, SOC_DROM_LOW, SOC_DROM_HIGH)) {
            return false;
        }
        std::memcpy(out, reinterpret_cast<const void*>(static_cast<std::uintptr_t>(address)), size);
        return true;
    }
    bool write(std::uint32_t address, const std::uint8_t* data, std::size_t size) override
    {
        if (!inRange(address, size, SOC_DRAM_LOW, SOC_DRAM_HIGH)) {
            return false;
        }
        std::memcpy(reinterpret_cast<void*>(static_cast<std::uintptr_t>(address)), data, size);
        return true;
    }

private:
    static bool inRange(std::uint32_t address, std::size_t size, std::uint32_t begin, std::uint32_t end)
    {
        return address >= begin && size <= end - address && address + size <= end;
    }
};

struct Datagram {
    sockaddr_in to;
    std::uint16_t length;
    std::uint8_t data[kMaxDatagram];
};

/// Packets with LEN/CTR header collected into datagrams; datagrams go to the sender task.
class UdpLink final : public IDtoTransmitter {
public:
    bool open(std::uint16_t port)
    {
        socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ < 0) {
            return false;
        }
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof address) != 0) {
            ::close(socket_);
            socket_ = -1;
            return false;
        }
        const int flags = ::fcntl(socket_, F_GETFL, 0);
        ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
        static StaticQueue_t queueBuffer;
        static std::uint8_t queueStorage[kQueueDepth * sizeof(Datagram)];
        queue_ = xQueueCreateStatic(kQueueDepth, sizeof(Datagram), queueStorage, &queueBuffer);
        xTaskCreatePinnedToCore(&UdpLink::sendTask, "xcp-tx", 4096, this, configMAX_PRIORITIES - 3, nullptr, 0);
        return true;
    }

    /// Next received datagram (non-blocking); remembers the master as destination.
    int receive(std::uint8_t* buffer, std::size_t size)
    {
        sockaddr_in from{};
        socklen_t length = sizeof from;
        const int n = ::recvfrom(socket_, buffer, size, 0, reinterpret_cast<sockaddr*>(&from), &length);
        if (n > 0) {
            pending_.to = from;
            hasMaster_ = true;
        }
        return n;
    }

    /// Adds a packet to the current datagram; false if it does not fit and the queue is full.
    bool queue(const std::uint8_t* packet, std::size_t size)
    {
        if (!hasMaster_) {
            return false;
        }
        if (pending_.length + 4 + size > kMaxDatagram && (!flush() || pending_.length + 4 + size > kMaxDatagram)) {
            return false;
        }
        std::uint8_t* p = pending_.data + pending_.length;
        p[0] = static_cast<std::uint8_t>(size & 0xFF);
        p[1] = static_cast<std::uint8_t>(size >> 8);
        p[2] = static_cast<std::uint8_t>(counter_ & 0xFF);
        p[3] = static_cast<std::uint8_t>(counter_ >> 8);
        ++counter_;
        std::memcpy(p + 4, packet, size);
        pending_.length = static_cast<std::uint16_t>(pending_.length + 4 + size);
        return true;
    }

    /// Hands the current datagram to the sender task (never blocks).
    bool flush()
    {
        if (pending_.length == 0) {
            return true;
        }
        if (xQueueSend(queue_, &pending_, 0) != pdTRUE) {
            ++sendErrors; // queue full: keep the datagram, retry next time
            return false;
        }
        pending_.length = 0;
        return true;
    }

    bool transmitDto(const std::uint8_t* packet, std::size_t size) override
    {
        if (!queue(packet, size)) {
            return false;
        }
        ++dtos;
        return true;
    }

    std::uint32_t dtos = 0;
    std::uint32_t sendErrors = 0;
    volatile std::uint32_t datagrams = 0;   // sender task
    volatile std::uint32_t sendRetries = 0; // sender task

private:
    static void sendTask(void* self)
    {
        auto& link = *static_cast<UdpLink*>(self);
        static Datagram d;
        for (;;) {
            if (xQueueReceive(link.queue_, &d, portMAX_DELAY) != pdTRUE) {
                continue;
            }
            // ENOMEM while the WLAN retransmits: try again a few times before giving up
            for (int attempt = 0; attempt < 20; ++attempt) {
                if (::sendto(link.socket_, d.data, d.length, 0, reinterpret_cast<const sockaddr*>(&d.to), sizeof d.to) >= 0) {
                    link.datagrams = link.datagrams + 1;
                    break;
                }
                link.sendRetries = link.sendRetries + 1;
                vTaskDelay(1);
            }
        }
    }

    int socket_ = -1;
    QueueHandle_t queue_ = nullptr;
    bool hasMaster_ = false;
    std::uint16_t counter_ = 0;
    Datagram pending_{};
};

struct State {
    Config config;
    EventChannel channels[kMaxEvents];
    SlaveConfig slaveConfig;
    EspMemory memory;
    UdpLink link;
    XcpSlave* slave = nullptr;
    SemaphoreHandle_t mutex = nullptr;
    StaticSemaphore_t mutexBuffer;
};

State& state()
{
    static State s;
    return s;
}

/// Slave clock: the FreeRTOS tick in ms (advances exactly with vTaskDelayUntil periods).
std::uint32_t nowMs()
{
    return static_cast<std::uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
}

/// The XCP time unit code of a cycle in ms (6 = 1 ms).
void cycleOf(std::uint16_t ms, EventChannel& c)
{
    c.cycle = static_cast<std::uint8_t>(ms <= 255 ? ms : ms / 10 <= 255 ? ms / 10 : ms / 100);
    c.timeUnit = static_cast<std::uint8_t>(ms <= 255 ? 6 : ms / 10 <= 255 ? 7 : 8);
}

void xcpTask(void*)
{
    State& s = state();
    static std::uint8_t rx[1500];
    TickType_t wake = xTaskGetTickCount();
    std::uint32_t lastFlush = 0;
    for (;;) {
        if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(1)) == pdFALSE) {
            wake = xTaskGetTickCount(); // too late: no catch-up burst
        }
        const std::uint32_t now = static_cast<std::uint32_t>(wake) * portTICK_PERIOD_MS;
        xSemaphoreTake(s.mutex, portMAX_DELAY);
        // commands: answered at once (own datagram)
        for (int n = s.link.receive(rx, sizeof rx); n > 0; n = s.link.receive(rx, sizeof rx)) {
            std::size_t pos = 0;
            while (pos + 4 <= static_cast<std::size_t>(n)) {
                const std::size_t len = rx[pos] | (rx[pos + 1] << 8);
                pos += 4;
                if (len == 0 || pos + len > static_cast<std::size_t>(n)) {
                    break;
                }
                std::uint8_t response[XcpSlave::kMaxCto];
                const std::size_t length = s.slave->process(rx + pos, len, response);
                if (length > 0) {
                    s.link.queue(response, length);
                }
                pos += len;
            }
            s.link.flush();
        }
        s.slave->setClock(now);
        if (s.slave->daqRunning()) {
            for (std::size_t e = 0; e < s.config.eventCount; ++e) {
                const Event& ev = s.config.events[e];
                if (ev.autoTrigger && ev.cycleMs > 0 && now % ev.cycleMs == 0) {
                    s.slave->triggerEvent(static_cast<std::uint16_t>(e), now, s.link);
                }
            }
        }
        // DTOs: one datagram per flush time
        const std::uint32_t flushMs = s.config.flushMs != nullptr && *s.config.flushMs > 0 ? *s.config.flushMs : 10U;
        if (now - lastFlush >= flushMs) {
            s.link.flush();
            lastFlush = now;
        }
        xSemaphoreGive(s.mutex);
    }
}

} // namespace

bool begin(const Config& config)
{
    State& s = state();
    if (s.slave != nullptr || config.eventCount > kMaxEvents) {
        return false;
    }
    s.config = config;
    for (std::size_t e = 0; e < config.eventCount; ++e) {
        s.channels[e].name = config.events[e].name;
        cycleOf(config.events[e].cycleMs, s.channels[e]);
    }
    SlaveConfig& c = s.slaveConfig;
    c.maxCto = 255;
    c.maxDto = 1024;
    c.driverVersion = 0x10;
    c.events = s.channels;
    c.eventCount = static_cast<std::uint16_t>(config.eventCount);
    c.timestampSize = 4;
    c.timestampUnit = 6; // 1 ms
    c.timestampTicks = 1;
    c.maxOdtEntrySize = 255;
    s.mutex = xSemaphoreCreateMutexStatic(&s.mutexBuffer);
    if (!s.link.open(config.port)) {
        return false;
    }
    static XcpSlave slave(s.memory, s.slaveConfig);
    s.slave = &slave;
    const unsigned priority = config.priority != 0 ? config.priority : configMAX_PRIORITIES - 2;
    xTaskCreatePinnedToCore(xcpTask, "xcp", 8192, nullptr, priority, nullptr, config.core);
    return true;
}

void trigger(std::uint16_t event)
{
    trigger(event, nowMs());
}

void trigger(std::uint16_t event, std::uint32_t timestampMs)
{
    State& s = state();
    if (s.slave == nullptr || event >= s.config.eventCount) {
        return;
    }
    xSemaphoreTake(s.mutex, portMAX_DELAY);
    if (s.slave->daqRunning()) {
        s.slave->triggerEvent(event, timestampMs, s.link);
    }
    xSemaphoreGive(s.mutex);
}

Stats stats()
{
    State& s = state();
    Stats st;
    if (s.slave == nullptr) {
        return st;
    }
    xSemaphoreTake(s.mutex, portMAX_DELAY);
    st.connected = s.slave->connected();
    st.daqRunning = s.slave->daqRunning();
    st.commands = s.slave->commandCount();
    st.overloads = s.slave->overloadCount();
    st.dtos = s.link.dtos;
    st.sendErrors = s.link.sendErrors;
    xSemaphoreGive(s.mutex);
    st.datagrams = s.link.datagrams;
    st.sendRetries = s.link.sendRetries;
    return st;
}

} // namespace x1xcp
