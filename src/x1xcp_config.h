#pragma once

// Sizes of the XCP slave for this library (the Arduino IDE has no build flags per
// library); -D options of the build (PlatformIO: library.json) win.

#ifndef X1_XCPSLAVE_MAX_CTO
#define X1_XCPSLAVE_MAX_CTO 255
#endif

#ifndef X1_XCPSLAVE_MAX_DTO
#define X1_XCPSLAVE_MAX_DTO 1024
#endif
