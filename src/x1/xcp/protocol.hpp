#pragma once

// XCP protocol constants (ASAM MCD-1 XCP V1.1, Part 2). Shared by master and simulator.

#include <cstdint>
// firmware without a C++ standard library (e.g. AVR): the constants only, no names
#if __has_include(<string_view>)
#include <string_view>
#define X1_XCP_PROTOCOL_NAMES 1
#endif

namespace x1::xcp {

/// Packet identifiers of slave-to-master packets. DAQ packets use 0x00..0xFB.
namespace pid {
constexpr std::uint8_t Res = 0xFF;
constexpr std::uint8_t Err = 0xFE;
constexpr std::uint8_t Ev = 0xFD;
constexpr std::uint8_t Serv = 0xFC;
constexpr std::uint8_t MaxDaq = 0xFB;
} // namespace pid

enum class Cmd : std::uint8_t {
    // Standard
    Connect = 0xFF,
    Disconnect = 0xFE,
    GetStatus = 0xFD,
    Synch = 0xFC,
    GetCommModeInfo = 0xFB,
    GetId = 0xFA,
    SetRequest = 0xF9,
    GetSeed = 0xF8,
    Unlock = 0xF7,
    SetMta = 0xF6,
    Upload = 0xF5,
    ShortUpload = 0xF4,
    BuildChecksum = 0xF3,
    TransportLayerCmd = 0xF2,
    UserCmd = 0xF1,
    // Calibration
    Download = 0xF0,
    DownloadNext = 0xEF,
    DownloadMax = 0xEE,
    ShortDownload = 0xED,
    ModifyBits = 0xEC,
    // Page switching
    SetCalPage = 0xEB,
    GetCalPage = 0xEA,
    GetPagProcessorInfo = 0xE9,
    GetSegmentInfo = 0xE8,
    GetPageInfo = 0xE7,
    SetSegmentMode = 0xE6,
    GetSegmentMode = 0xE5,
    CopyCalPage = 0xE4,
    // Data acquisition
    ClearDaqList = 0xE3,
    SetDaqPtr = 0xE2,
    WriteDaq = 0xE1,
    SetDaqListMode = 0xE0,
    GetDaqListMode = 0xDF,
    StartStopDaqList = 0xDE,
    StartStopSynch = 0xDD,
    GetDaqClock = 0xDC,
    ReadDaq = 0xDB,
    GetDaqProcessorInfo = 0xDA,
    GetDaqResolutionInfo = 0xD9,
    GetDaqListInfo = 0xD8,
    GetDaqEventInfo = 0xD7,
    FreeDaq = 0xD6,
    AllocDaq = 0xD5,
    AllocOdt = 0xD4,
    AllocOdtEntry = 0xD3,
    WriteDaqMultiple = 0xC7,
    // Programming
    ProgramStart = 0xD2,
    ProgramClear = 0xD1,
    Program = 0xD0,
    ProgramReset = 0xCF,
    GetPgmProcessorInfo = 0xCE,
    GetSectorInfo = 0xCD,
    ProgramPrepare = 0xCC,
    ProgramFormat = 0xCB,
    ProgramNext = 0xCA,
    ProgramMax = 0xC9,
    ProgramVerify = 0xC8,
};

#ifdef X1_XCP_PROTOCOL_NAMES
std::string_view toString(Cmd cmd) noexcept;
#endif

/// Error codes of ERR packets.
enum class ErrCode : std::uint8_t {
    CmdSynch = 0x00,
    CmdBusy = 0x10,
    DaqActive = 0x11,
    PgmActive = 0x12,
    CmdUnknown = 0x20,
    CmdSyntax = 0x21,
    OutOfRange = 0x22,
    WriteProtected = 0x23,
    AccessDenied = 0x24,
    AccessLocked = 0x25,
    PageNotValid = 0x26,
    ModeNotValid = 0x27,
    SegmentNotValid = 0x28,
    Sequence = 0x29,
    DaqConfig = 0x2A,
    MemoryOverflow = 0x30,
    Generic = 0x31,
    Verify = 0x32,
    ResourceTemporaryNotAccessible = 0x33,
};

#ifdef X1_XCP_PROTOCOL_NAMES
std::string_view toString(ErrCode code) noexcept;
#endif

/// Event codes of EV packets.
enum class EvCode : std::uint8_t {
    ResumeMode = 0x00,
    ClearDaq = 0x01,
    StoreDaq = 0x02,
    StoreCal = 0x03,
    CmdPending = 0x05,
    DaqOverload = 0x06,
    SessionTerminated = 0x07,
    TimeSync = 0x08,
    StimTimeout = 0x09,
    Sleep = 0x0A,
    WakeUp = 0x0B,
    User = 0xFE,
    Transport = 0xFF,
};

#ifdef X1_XCP_PROTOCOL_NAMES
std::string_view toString(EvCode code) noexcept;
#endif

/// Service request codes of SERV packets.
enum class ServCode : std::uint8_t {
    Reset = 0x00,
    Text = 0x01,
};

enum class ConnectMode : std::uint8_t { Normal = 0x00, UserDefined = 0x01 };

/// RESOURCE (CONNECT) and resource protection status (GET_STATUS) bits.
namespace resource {
constexpr std::uint8_t CalPag = 0x01;
constexpr std::uint8_t Daq = 0x04;
constexpr std::uint8_t Stim = 0x08;
constexpr std::uint8_t Pgm = 0x10;
} // namespace resource

/// COMM_MODE_BASIC bits (CONNECT).
namespace comm_mode_basic {
constexpr std::uint8_t ByteOrderMotorola = 0x01;
constexpr std::uint8_t AddressGranularityMask = 0x06;
constexpr std::uint8_t SlaveBlockMode = 0x40;
constexpr std::uint8_t Optional = 0x80;
} // namespace comm_mode_basic

/// COMM_MODE_OPTIONAL bits (GET_COMM_MODE_INFO).
namespace comm_mode_optional {
constexpr std::uint8_t MasterBlockMode = 0x01;
constexpr std::uint8_t InterleavedMode = 0x02;
} // namespace comm_mode_optional

/// Current session status bits (GET_STATUS).
namespace session_status {
constexpr std::uint8_t StoreCalReq = 0x01;
constexpr std::uint8_t StoreDaqReq = 0x04;
constexpr std::uint8_t ClearDaqReq = 0x08;
constexpr std::uint8_t DaqRunning = 0x40;
constexpr std::uint8_t Resume = 0x80;
} // namespace session_status

enum class ByteOrder : std::uint8_t { Intel, Motorola };

/// Address granularity: size of one addressable element in bytes (1, 2 or 4).
enum class AddressGranularity : std::uint8_t { Byte = 1, Word = 2, DWord = 4 };

/// Timeout classes t1..t7 of the error handling matrix (Part 2, 1.7.2).
enum class TimeoutClass : std::uint8_t { T1, T2, T3, T4, T5, T6, T7 };

} // namespace x1::xcp
