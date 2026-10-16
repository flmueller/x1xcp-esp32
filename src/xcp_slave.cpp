#include "x1/xcpslave/xcp_slave.hpp"

#include "x1/xcp/protocol.hpp"

namespace x1::xcpslave {

using x1::xcp::Cmd;
using x1::xcp::ErrCode;
namespace pid = x1::xcp::pid;

namespace {

std::uint16_t readU16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t readU32(const std::uint8_t* p)
{
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

void writeU16(std::uint8_t* p, std::uint16_t v)
{
    p[0] = static_cast<std::uint8_t>(v & 0xFF);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}

void writeU32(std::uint8_t* p, std::uint32_t v)
{
    for (int i = 0; i < 4; ++i) {
        p[i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
    }
}

constexpr std::uint8_t code(ErrCode e)
{
    return static_cast<std::uint8_t>(e);
}

std::size_t length(const char* text)
{
    std::size_t n = 0;
    while (text[n] != '\0') {
        ++n;
    }
    return n;
}

// SET_DAQ_LIST_MODE mode bits
constexpr std::uint8_t kModeAlternating = 0x01;
constexpr std::uint8_t kModeDirectionStim = 0x02;
constexpr std::uint8_t kModeTimestamp = 0x10;
constexpr std::uint8_t kModePidOff = 0x20;
// GET_DAQ_LIST_MODE additional status bits
constexpr std::uint8_t kModeSelected = 0x01;
constexpr std::uint8_t kModeRunning = 0x40;

// DAQ_PROPERTIES: dynamic, prescaler, timestamp, overload indication in PID MSB
constexpr std::uint8_t kDaqProperties = 0x01 | 0x02 | 0x10 | 0x40;
constexpr std::uint8_t kOverloadMsb = 0x80;

} // namespace

std::size_t XcpSlave::error(std::uint8_t errorCode, std::uint8_t* response) const
{
    response[0] = pid::Err;
    response[1] = errorCode;
    return 2;
}

std::size_t XcpSlave::ok(std::uint8_t* response) const
{
    response[0] = pid::Res;
    return 1;
}

bool XcpSlave::daqRunning() const
{
    for (std::uint16_t i = 0; i < listCount_; ++i) {
        if (lists_[i].running) {
            return true;
        }
    }
    return false;
}

void XcpSlave::stopAll()
{
    for (auto& list : lists_) {
        list.running = false;
        list.selected = false;
    }
}

void XcpSlave::freeDaq()
{
    stopAll();
    for (auto& list : lists_) {
        list = DaqList{};
    }
    for (std::uint16_t i = 0; i < odtCount_; ++i) {
        odts_[i] = Odt{}; // ALLOC_ODT_ENTRY rejects ODTs that still have entries
    }
    listCount_ = 0;
    odtCount_ = 0;
    entryCount_ = 0;
    allocState_ = AllocState::Freed;
    ptrValid_ = false;
    overloadPending_ = false;
}

std::size_t XcpSlave::upload(std::uint8_t elements, std::uint8_t* response)
{
    if (elements == 0 || elements > config_.maxCto - 1) {
        return error(code(ErrCode::OutOfRange), response);
    }
    if (mta_ >= kEventNameBase) {
        // Event channel name after GET_DAQ_EVENT_INFO: kEventNameBase + event * 0x100 + offset
        const std::uint32_t event = (mta_ - kEventNameBase) >> 8;
        const std::uint32_t offset = (mta_ - kEventNameBase) & 0xFF;
        if (event >= config_.eventCount || offset + elements > length(config_.events[event].name)) {
            return error(code(ErrCode::AccessDenied), response);
        }
        for (std::uint8_t i = 0; i < elements; ++i) {
            response[1 + i] = static_cast<std::uint8_t>(config_.events[event].name[offset + i]);
        }
    } else if (!memory_.read(mta_, response + 1, elements)) {
        return error(code(ErrCode::AccessDenied), response);
    }
    response[0] = pid::Res;
    mta_ += elements;
    return 1U + elements;
}

std::size_t XcpSlave::process(const std::uint8_t* cmd, std::size_t len, std::uint8_t* response)
{
    if (len == 0) {
        return 0;
    }
    ++commands_;
    const auto command = static_cast<Cmd>(cmd[0]);
    if (!connected_ && command != Cmd::Connect) {
        return 0; // a disconnected slave only reacts to CONNECT
    }

    switch (command) {
    case Cmd::Connect:
        if (len < 2 || cmd[1] > 1) {
            return error(code(ErrCode::OutOfRange), response);
        }
        if (!connected_) {
            freeDaq();
        }
        connected_ = true;
        response[0] = pid::Res;
        response[1] = static_cast<std::uint8_t>(x1::xcp::resource::CalPag |
                                                (config_.eventCount > 0 ? x1::xcp::resource::Daq : 0));
        response[2] = x1::xcp::comm_mode_basic::Optional; // Intel, AG = BYTE, GET_COMM_MODE_INFO available
        response[3] = config_.maxCto;
        writeU16(response + 4, config_.maxDto);
        response[6] = 0x01; // protocol layer version
        response[7] = 0x01; // transport layer version
        return 8;

    case Cmd::Disconnect:
        freeDaq();
        connected_ = false;
        return ok(response);

    case Cmd::GetStatus:
        response[0] = pid::Res;
        response[1] = daqRunning() ? x1::xcp::session_status::DaqRunning : 0;
        response[2] = 0; // no protection
        response[3] = 0;
        writeU16(response + 4, 0); // session configuration id
        return 6;

    case Cmd::Synch: return error(code(ErrCode::CmdSynch), response);

    case Cmd::GetCommModeInfo:
        response[0] = pid::Res;
        response[1] = 0;
        response[2] = 0; // no master block mode, no interleaved mode
        response[3] = 0;
        response[4] = 0; // MAX_BS
        response[5] = 0; // MIN_ST
        response[6] = 0; // queue size
        response[7] = config_.driverVersion;
        return 8;

    case Cmd::SetMta:
        if (len < 8) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        if (cmd[3] != 0) {
            return error(code(ErrCode::OutOfRange), response);
        }
        mta_ = readU32(cmd + 4);
        return ok(response);

    case Cmd::Upload:
        if (len < 2) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        return upload(cmd[1], response);

    case Cmd::ShortUpload: {
        if (len < 8) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        if (cmd[3] != 0) {
            return error(code(ErrCode::OutOfRange), response);
        }
        const std::uint8_t elements = cmd[1];
        if (shortUploadBug_ && elements > 0 && elements < config_.maxCto) {
            // Fault injection: data that does not depend on the address (seen on a real ECU, REQ-XCP-017).
            response[0] = pid::Res;
            for (std::uint8_t i = 0; i < elements; ++i) {
                response[1 + i] = (i % 2 == 0) ? 0x00 : 0xBF;
            }
            return 1U + elements;
        }
        mta_ = readU32(cmd + 4);
        return upload(elements, response);
    }

    case Cmd::Download: {
        if (len < 2) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        const std::size_t elements = cmd[1];
        if (elements == 0 || elements > static_cast<std::size_t>(config_.maxCto - 2)) {
            return error(code(ErrCode::OutOfRange), response);
        }
        if (len < 2 + elements) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        if (!memory_.write(mta_, cmd + 2, elements)) {
            return error(code(ErrCode::AccessDenied), response);
        }
        mta_ += static_cast<std::uint32_t>(elements);
        return ok(response);
    }

    default:
        if (config_.eventCount > 0) {
            return processDaq(cmd, len, response);
        }
        return error(code(ErrCode::CmdUnknown), response);
    }
}

bool XcpSlave::listFitsDto(const DaqList& list) const
{
    for (std::uint8_t o = 0; o < list.odtCount; ++o) {
        const Odt& odt = odts_[list.firstOdt + o];
        std::size_t size = 1; // PID
        if (o == 0 && (list.mode & kModeTimestamp) != 0) {
            size += config_.timestampSize;
        }
        for (std::uint8_t e = 0; e < odt.entryCount; ++e) {
            size += entries_[odt.firstEntry + e].size;
        }
        if (size > config_.maxDto || size > kMaxDto) {
            return false;
        }
    }
    return list.odtCount > 0;
}

std::size_t XcpSlave::processDaq(const std::uint8_t* cmd, std::size_t len, std::uint8_t* response)
{
    switch (static_cast<Cmd>(cmd[0])) {
    case Cmd::GetDaqProcessorInfo:
        response[0] = pid::Res;
        response[1] = kDaqProperties;
        writeU16(response + 2, listCount_); // MAX_DAQ: MIN_DAQ (0) + allocated lists
        writeU16(response + 4, config_.eventCount);
        response[6] = 0;    // MIN_DAQ
        response[7] = 0x00; // DAQ_KEY_BYTE: default optimisation, address extension free, absolute ODT number
        return 8;

    case Cmd::GetDaqResolutionInfo:
        if (!config_.daqInfoCommands) {
            return error(code(ErrCode::CmdUnknown), response);
        }
        response[0] = pid::Res;
        response[1] = 1; // granularity DAQ
        response[2] = config_.maxOdtEntrySize;
        response[3] = 1; // granularity STIM
        response[4] = config_.maxOdtEntrySize;
        response[5] = static_cast<std::uint8_t>((config_.timestampUnit << 4) | (config_.timestampFixed ? 0x08 : 0) |
                                                (config_.timestampSize & 0x07));
        writeU16(response + 6, config_.timestampTicks);
        return 8;

    case Cmd::GetDaqEventInfo: {
        if (!config_.daqInfoCommands) {
            return error(code(ErrCode::CmdUnknown), response);
        }
        if (len < 4) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        const std::uint16_t event = readU16(cmd + 2);
        if (event >= config_.eventCount) {
            return error(code(ErrCode::OutOfRange), response);
        }
        const EventChannel& e = config_.events[event];
        const std::size_t nameLength = length(e.name);
        response[0] = pid::Res;
        response[1] = e.properties;
        response[2] = e.maxDaqList;
        response[3] = static_cast<std::uint8_t>(nameLength > 255 ? 255 : nameLength);
        response[4] = e.cycle;
        response[5] = e.timeUnit;
        response[6] = e.priority;
        mta_ = kEventNameBase + (static_cast<std::uint32_t>(event) << 8);
        return 7;
    }

    case Cmd::GetDaqClock:
        response[0] = pid::Res;
        response[1] = 0;
        response[2] = 0;
        response[3] = 0;
        writeU32(response + 4, clock_);
        return 8;

    case Cmd::FreeDaq:
        freeDaq();
        return ok(response);

    case Cmd::AllocDaq: {
        if (len < 4) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        if (allocState_ == AllocState::Odt || allocState_ == AllocState::Entry) {
            return error(code(ErrCode::Sequence), response);
        }
        const std::uint16_t count = readU16(cmd + 2);
        if (count > kMaxDaqLists) {
            return error(code(ErrCode::MemoryOverflow), response);
        }
        listCount_ = count;
        allocState_ = AllocState::Daq;
        return ok(response);
    }

    case Cmd::AllocOdt: {
        if (len < 5) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        if (allocState_ == AllocState::Freed || allocState_ == AllocState::Entry) {
            return error(code(ErrCode::Sequence), response);
        }
        const std::uint16_t list = readU16(cmd + 2);
        const std::uint8_t count = cmd[4];
        if (list >= listCount_ || lists_[list].odtCount != 0) {
            return error(code(ErrCode::OutOfRange), response);
        }
        if (odtCount_ + count > kMaxOdts) {
            return error(code(ErrCode::MemoryOverflow), response);
        }
        lists_[list].firstOdt = odtCount_;
        lists_[list].odtCount = count;
        odtCount_ = static_cast<std::uint16_t>(odtCount_ + count);
        allocState_ = AllocState::Odt;
        return ok(response);
    }

    case Cmd::AllocOdtEntry: {
        if (len < 6) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        if (allocState_ == AllocState::Freed || allocState_ == AllocState::Daq) {
            return error(code(ErrCode::Sequence), response);
        }
        const std::uint16_t list = readU16(cmd + 2);
        const std::uint8_t odt = cmd[4];
        const std::uint8_t count = cmd[5];
        if (list >= listCount_ || odt >= lists_[list].odtCount || odts_[lists_[list].firstOdt + odt].entryCount != 0) {
            return error(code(ErrCode::OutOfRange), response);
        }
        if (entryCount_ + count > kMaxOdtEntries) {
            return error(code(ErrCode::MemoryOverflow), response);
        }
        Odt& o = odts_[lists_[list].firstOdt + odt];
        o.firstEntry = entryCount_;
        o.entryCount = count;
        entryCount_ = static_cast<std::uint16_t>(entryCount_ + count);
        allocState_ = AllocState::Entry;
        return ok(response);
    }

    case Cmd::SetDaqPtr: {
        if (len < 6) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        const std::uint16_t list = readU16(cmd + 2);
        const std::uint8_t odt = cmd[4];
        const std::uint8_t entry = cmd[5];
        if (list >= listCount_ || odt >= lists_[list].odtCount ||
            entry >= odts_[lists_[list].firstOdt + odt].entryCount) {
            return error(code(ErrCode::OutOfRange), response);
        }
        if (lists_[list].running) {
            return error(code(ErrCode::DaqActive), response);
        }
        ptrList_ = list;
        ptrOdt_ = odt;
        ptrEntry_ = entry;
        ptrValid_ = true;
        return ok(response);
    }

    case Cmd::WriteDaq: {
        if (len < 8) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        const Odt& odt = odts_[lists_[ptrList_].firstOdt + ptrOdt_];
        if (!ptrValid_ || ptrEntry_ >= odt.entryCount) {
            return error(code(ErrCode::Sequence), response);
        }
        const std::uint8_t bitOffset = cmd[1];
        const std::uint8_t size = cmd[2];
        const std::uint32_t address = readU32(cmd + 4);
        if (bitOffset != 0xFF || cmd[3] != 0 || size == 0 || size > config_.maxOdtEntrySize) {
            return error(code(ErrCode::OutOfRange), response);
        }
        std::uint8_t probe[8];
        if (!memory_.read(address, probe, size)) {
            return error(code(ErrCode::AccessDenied), response);
        }
        OdtEntry& entry = entries_[odt.firstEntry + ptrEntry_]; // member by member: also C++11 (Arduino IDE, AVR)
        entry.address = address;
        entry.size = size;
        ++ptrEntry_; // post increment within the ODT
        return ok(response);
    }

    case Cmd::SetDaqListMode: {
        if (len < 8) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        const std::uint8_t mode = cmd[1];
        const std::uint16_t list = readU16(cmd + 2);
        const std::uint16_t event = readU16(cmd + 4);
        const std::uint8_t prescaler = cmd[6];
        if (list >= listCount_ || event >= config_.eventCount || prescaler == 0) {
            return error(code(ErrCode::OutOfRange), response);
        }
        if ((mode & (kModeAlternating | kModeDirectionStim | kModePidOff)) != 0) {
            return error(code(ErrCode::OutOfRange), response);
        }
        if (config_.timestampFixed && (mode & kModeTimestamp) == 0) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        if (lists_[list].running) {
            return error(code(ErrCode::DaqActive), response);
        }
        DaqList& l = lists_[list];
        l.mode = mode & kModeTimestamp;
        l.event = event;
        l.prescaler = prescaler;
        l.prescalerCounter = 0;
        l.priority = cmd[7];
        return ok(response);
    }

    case Cmd::GetDaqListMode: {
        if (len < 4) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        const std::uint16_t list = readU16(cmd + 2);
        if (list >= listCount_) {
            return error(code(ErrCode::OutOfRange), response);
        }
        const DaqList& l = lists_[list];
        response[0] = pid::Res;
        response[1] = static_cast<std::uint8_t>(l.mode | (l.selected ? kModeSelected : 0) | (l.running ? kModeRunning : 0));
        response[2] = 0;
        response[3] = 0;
        writeU16(response + 4, l.event);
        response[6] = l.prescaler;
        response[7] = l.priority;
        return 8;
    }

    case Cmd::StartStopDaqList: {
        if (len < 4) {
            return error(code(ErrCode::CmdSyntax), response);
        }
        const std::uint8_t mode = cmd[1];
        const std::uint16_t list = readU16(cmd + 2);
        if (list >= listCount_ || mode > 2) {
            return error(code(ErrCode::OutOfRange), response);
        }
        DaqList& l = lists_[list];
        if (mode != 0 && !listFitsDto(l)) {
            return error(code(ErrCode::DaqConfig), response);
        }
        if (mode == 0) {
            l.running = false;
        } else if (mode == 1) {
            l.running = true;
            l.prescalerCounter = 0;
        } else {
            l.selected = true;
        }
        response[0] = pid::Res;
        response[1] = static_cast<std::uint8_t>(l.firstOdt); // FIRST_PID (absolute ODT number)
        return 2;
    }

    case Cmd::StartStopSynch: {
        if (len < 2 || cmd[1] > 2) {
            return error(code(ErrCode::OutOfRange), response);
        }
        const std::uint8_t mode = cmd[1];
        for (std::uint16_t i = 0; i < listCount_; ++i) {
            DaqList& l = lists_[i];
            if (mode == 0) {
                l.running = false;
            } else if (l.selected) {
                l.running = mode == 1;
                l.prescalerCounter = 0;
            }
            if (mode != 0) {
                l.selected = false;
            }
        }
        return ok(response);
    }

    default: return error(code(ErrCode::CmdUnknown), response);
    }
}

void XcpSlave::triggerEvent(std::uint16_t event, std::uint32_t timestamp, IDtoTransmitter& transmitter)
{
    for (std::uint16_t i = 0; i < listCount_; ++i) {
        DaqList& list = lists_[i];
        if (!list.running || list.event != event) {
            continue;
        }
        if (++list.prescalerCounter < list.prescaler) {
            continue;
        }
        list.prescalerCounter = 0;

        for (std::uint8_t o = 0; o < list.odtCount; ++o) {
            const Odt& odt = odts_[list.firstOdt + o];
            std::uint8_t packet[kMaxDto];
            std::size_t size = 0;
            packet[size++] = static_cast<std::uint8_t>((list.firstOdt + o) | (overloadPending_ ? kOverloadMsb : 0));
            if (o == 0 && (list.mode & kModeTimestamp) != 0) {
                for (std::uint8_t b = 0; b < config_.timestampSize; ++b) {
                    packet[size++] = static_cast<std::uint8_t>((timestamp >> (8 * b)) & 0xFF);
                }
            }
            for (std::uint8_t e = 0; e < odt.entryCount; ++e) {
                const OdtEntry& entry = entries_[odt.firstEntry + e];
                if (!memory_.read(entry.address, packet + size, entry.size)) {
                    for (std::uint8_t b = 0; b < entry.size; ++b) {
                        packet[size + b] = 0;
                    }
                }
                size += entry.size;
            }
            if (transmitter.transmitDto(packet, size)) {
                overloadPending_ = false;
            } else {
                // Queue full: drop the rest of this cycle, flag the next transmitted DTO.
                overloadPending_ = true;
                ++overloads_;
                break;
            }
        }
    }
}

} // namespace x1::xcpslave
