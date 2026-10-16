#pragma once

// Platform independent XCP slave protocol layer with dynamic DAQ (shared by the simulator and the
// XMC4700 test ECU firmware). No heap, no exceptions, no threads: the owner serializes calls of
// process() and triggerEvent().
//
// Behaviour of a typical production ECU (REQ-XCP-005/015, REQ-FW-003): Intel byte order, AG = BYTE,
// dynamic DAQ, identification field "absolute ODT number", overload indication in the PID MSB,
// switchable timestamp in the first ODT of a DAQ list.

#include <cstddef>
#include <cstdint>

// The Arduino libraries bring their sizes in x1xcp_config.h (the Arduino IDE has no build flags
// per library); -D options of the build (PlatformIO) still win.
#if defined(__has_include)
#if __has_include("x1xcp_config.h")
#include "x1xcp_config.h"
#endif
#endif

// Largest CTO/DTO the slave can handle (buffer sizes on the stack). CAN builds keep 8 bytes
// (small stacks, e.g. the XMC4700 firmware); Ethernet builds define larger values, e.g.
// X1_XCPSLAVE_MAX_CTO=255 and X1_XCPSLAVE_MAX_DTO=1024 (the host library and the ESP32).
#ifndef X1_XCPSLAVE_MAX_CTO
#define X1_XCPSLAVE_MAX_CTO 8
#endif
#ifndef X1_XCPSLAVE_MAX_DTO
#define X1_XCPSLAVE_MAX_DTO 8
#endif
// Size of the dynamic DAQ tables (RAM: about 12, 4 and 8 bytes per list, ODT and entry). Small
// controllers reduce them, e.g. the ATmega328P (2 KB RAM): 4 lists, 12 ODTs, 48 entries.
#ifndef X1_XCPSLAVE_MAX_DAQ_LISTS
#define X1_XCPSLAVE_MAX_DAQ_LISTS 16
#endif
#ifndef X1_XCPSLAVE_MAX_ODTS
#define X1_XCPSLAVE_MAX_ODTS 0x7C
#endif
#ifndef X1_XCPSLAVE_MAX_ODT_ENTRIES
#define X1_XCPSLAVE_MAX_ODT_ENTRIES 512
#endif

namespace x1::xcpslave {

/// Memory access in XCP addresses (AG = BYTE, address extension 0).
class IMemory {
public:
    virtual bool read(std::uint32_t address, std::uint8_t* out, std::size_t size) = 0;
    virtual bool write(std::uint32_t address, const std::uint8_t* data, std::size_t size) = 0;

protected:
    // Not deleted through the interface; non-virtual so that firmware builds need no heap.
    ~IMemory() = default;
};

/// Sends DAQ packets (DTOs). Returns false if the packet could not be queued (overload).
class IDtoTransmitter {
public:
    virtual bool transmitDto(const std::uint8_t* packet, std::size_t size) = 0;

protected:
    ~IDtoTransmitter() = default;
};

struct EventChannel {
    const char* name = "";
    std::uint8_t cycle = 0;    ///< 0 = not cyclic
    std::uint8_t timeUnit = 6; ///< XCP time unit code (6 = 1 ms, 7 = 10 ms, 8 = 100 ms)
    std::uint8_t maxDaqList = 1;
    std::uint8_t priority = 0;
    std::uint8_t properties = 0; ///< DAQ_EVENT_PROPERTIES; some ECUs report 0 (REQ-XCP-015)
};

struct SlaveConfig {
    std::uint8_t maxCto = 8;
    std::uint16_t maxDto = 8;
    std::uint8_t driverVersion = 0x10;

    // DAQ (disabled if eventCount == 0)
    const EventChannel* events = nullptr;
    std::uint16_t eventCount = 0;
    std::uint8_t timestampSize = 2;    ///< bytes: 1, 2 or 4
    std::uint8_t timestampUnit = 6;    ///< 6 = 1 ms
    std::uint16_t timestampTicks = 1;
    bool timestampFixed = false;
    std::uint8_t maxOdtEntrySize = 7;
    /// Answer the optional GET_DAQ_RESOLUTION_INFO and GET_DAQ_EVENT_INFO (false: ERR_CMD_UNKNOWN,
    /// a slave that leaves these values to the A2L or manual settings, REQ-XCP-020).
    bool daqInfoCommands = true;
};

class XcpSlave {
public:
    static constexpr std::size_t kMaxCto = X1_XCPSLAVE_MAX_CTO;
    static constexpr std::size_t kMaxDaqLists = X1_XCPSLAVE_MAX_DAQ_LISTS;
    static constexpr std::size_t kMaxOdts = X1_XCPSLAVE_MAX_ODTS; ///< at most PIDs 0x00..0x7B, MSB = overload indication
    static constexpr std::size_t kMaxOdtEntries = X1_XCPSLAVE_MAX_ODT_ENTRIES;
    static constexpr std::size_t kMaxDto = X1_XCPSLAVE_MAX_DTO;
    static_assert(kMaxCto >= 8 && kMaxCto <= 255, "MAX_CTO 8..255");
    static_assert(kMaxDto >= 8 && kMaxDto <= 65535, "MAX_DTO 8..65535");
    static_assert(kMaxDaqLists >= 1 && kMaxOdts >= 1 && kMaxOdts <= 0x7C && kMaxOdtEntries >= 1, "DAQ table sizes");
    /// MTA of event channel names after GET_DAQ_EVENT_INFO (read with UPLOAD).
    static constexpr std::uint32_t kEventNameBase = 0xFFFF0000U;

    XcpSlave(IMemory& memory, const SlaveConfig& config) : memory_(memory), config_(config)
    {
        // never more than the buffers can hold
        config_.maxCto = static_cast<std::uint8_t>(config_.maxCto > kMaxCto ? kMaxCto : config_.maxCto);
        config_.maxDto = static_cast<std::uint16_t>(config_.maxDto > kMaxDto ? kMaxDto : config_.maxDto);
    }

    /// Processes one command packet. Writes the response to `response` (kMaxCto bytes) and
    /// returns its length; 0 = no response (e.g. not connected).
    std::size_t process(const std::uint8_t* command, std::size_t length, std::uint8_t* response);

    /// Called by the application when event channel `event` occurs; samples and sends all running
    /// DAQ lists of this event. `timestamp` is the slave clock in timestamp ticks.
    void triggerEvent(std::uint16_t event, std::uint32_t timestamp, IDtoTransmitter& transmitter);

    /// Current slave clock for GET_DAQ_CLOCK (set by the application, e.g. every millisecond).
    void setClock(std::uint32_t timestamp) { clock_ = timestamp; }

    bool connected() const { return connected_; }
    bool daqRunning() const;
    std::uint32_t commandCount() const { return commands_; }
    std::uint32_t overloadCount() const { return overloads_; }

    /// Fault injection: answer SHORT_UPLOAD with address-independent data, as seen on a real ECU (REQ-XCP-017).
    void setShortUploadBug(bool enabled) { shortUploadBug_ = enabled; }

private:
    enum class AllocState : std::uint8_t { Freed, Daq, Odt, Entry };

    struct DaqList {
        std::uint16_t firstOdt = 0;
        std::uint8_t odtCount = 0;
        std::uint8_t mode = 0;
        std::uint16_t event = 0;
        std::uint8_t prescaler = 1;
        std::uint8_t prescalerCounter = 0;
        std::uint8_t priority = 0;
        bool selected = false;
        bool running = false;
    };
    struct Odt {
        std::uint16_t firstEntry = 0;
        std::uint8_t entryCount = 0;
    };
    struct OdtEntry {
        std::uint32_t address = 0;
        std::uint8_t size = 0;
    };

    std::size_t error(std::uint8_t code, std::uint8_t* response) const;
    std::size_t ok(std::uint8_t* response) const;
    std::size_t upload(std::uint8_t elements, std::uint8_t* response);
    std::size_t processDaq(const std::uint8_t* cmd, std::size_t length, std::uint8_t* response);
    bool listFitsDto(const DaqList& list) const;
    void freeDaq();
    void stopAll();

    IMemory& memory_;
    SlaveConfig config_;
    bool connected_ = false;
    bool shortUploadBug_ = false;
    std::uint32_t mta_ = 0;
    std::uint32_t commands_ = 0;
    std::uint32_t clock_ = 0;

    // Dynamic DAQ configuration
    DaqList lists_[kMaxDaqLists];
    Odt odts_[kMaxOdts];
    OdtEntry entries_[kMaxOdtEntries];
    std::uint16_t listCount_ = 0;
    std::uint16_t odtCount_ = 0;
    std::uint16_t entryCount_ = 0;
    AllocState allocState_ = AllocState::Freed;
    std::uint16_t ptrList_ = 0;
    std::uint8_t ptrOdt_ = 0;
    std::uint8_t ptrEntry_ = 0;
    bool ptrValid_ = false;
    bool overloadPending_ = false;
    std::uint32_t overloads_ = 0;
};

} // namespace x1::xcpslave
