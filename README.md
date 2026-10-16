# x1xcp_esp32 – XCP on UDP/IP for ESP32

Adds measurement (dynamic DAQ, up to 1 ms) and calibration of global variables over the network
(LAN, XCP on UDP/IP) to an ESP32 firmware, for XCP-Ray or any XCP master. The A2L file is
generated from the ELF file after every build: addresses and data types from the debug
information, the selection, units and scaling from a short variable list.

Tested: NodeMCU-32S (ESP32), PlatformIO with Arduino-ESP32 3.3 (pioarduino), 60 s with a 1 ms
DAQ list over the network without loss. It uses only FreeRTOS, lwIP sockets and the memory map of
the chip (`soc/soc.h`), so other ESP32 chips (S2, S3, C3, …) and the ESP-IDF framework should work
as well, but are untested.

Part of the free kits of XCP-Ray, a measurement and calibration tool. The library is MIT
licensed; XCP-Ray itself and its A2L generator `x1-elf2a2l` are free to use, but not open source.

## Installation

- **Arduino IDE:** Sketch > Include Library > Add .ZIP Library… with the zip of this repository
  (Code > Download ZIP on GitHub, or a release). Then File > Examples > x1xcp_esp32 > Basic.
- **PlatformIO:** in `platformio.ini`: `lib_deps = https://github.com/flmueller/x1xcp-esp32.git`

## Contents

| | |
|---|---|
| `src/` | the library: `x1xcp.h` (API), the portable XCP slave of XCP-Ray, `x1xcp_config.h` (sizes) |
| `examples/Basic/` | a sketch extended with XCP (the lines marked "XCP") |
| `extras/a2l/variables.example.json` | template of the variable list for the A2L file |
| `extras/scripts/x1_a2l.py` | PlatformIO post-build step: the A2L file after every build |

## Steps

1. **Library:** install it (above) and add `#include <x1xcp.h>` (see the example and step 4).

2. **A2L file:** XCP-Ray needs an A2L file with the addresses of the variables; `x1-elf2a2l`
   makes it from the ELF file of the build and a short variable list (step 3). `x1-elf2a2l` is
   in the `bin/` folder of XCP-Ray (Linux and Windows). Make the A2L again after every change of
   the firmware: the addresses move.

   **Arduino IDE:** Sketch > Export Compiled Binary writes the ELF file into the sketch folder,
   `build/<board>/<sketch>.ino.elf`. Then, in the sketch folder:

   ```sh
   x1-elf2a2l -v variables.json -o <sketch>.a2l build/<board>/<sketch>.ino.elf
   ```

   If `x1-elf2a2l` reports a variable without debug information (always for AVR boards: the
   Arduino IDE builds them with LTO, which drops it), give the variable its type in the list,
   e.g. `"type": "uint16_t"`, `"float"` or `"bool"`; the address then comes from the symbol
   table. Curves need the `"type"` of the values and, inside `"curve"`, of the axis; maps need
   the debug information (PlatformIO).

   **PlatformIO:** copy `extras/scripts/x1_a2l.py` into `scripts/` of the project, `x1-elf2a2l`
   into `tools/` (or set `X1_ELF2A2L`), and add to `platformio.ini`:

   ```ini
   extra_scripts = post:scripts/x1_a2l.py
   ; optional, these are the defaults:
   custom_x1_variables = a2l/variables.json
   custom_x1_a2l = <project folder name>.a2l
   ```

   Every `pio run` then writes the A2L file.

3. **Variable list:** copy `extras/a2l/variables.example.json` to `variables.json` (Arduino IDE: next to the sketch; PlatformIO: `a2l/variables.json`) and list the
   variables to measure (`"event"`) and to calibrate (`"parameter": true`), with units, scaling
   and texts. The `"events"` must match the firmware (step 4). Structures and arrays are expanded
   to their members; enumerations get their texts from the ELF.

   **Characteristic curves** (y over x, edited as a graph in XCP-Ray): two global arrays of the
   same length, the axis points (ascending) and the values, e.g.
   `volatile float gain_x[8]; volatile float gain[8];` and in the list

   ```json
   { "name": "gain", "unit": "%", "min": 0, "max": 100,
     "curve": { "axis": "gain_x", "input": "rpm", "unit": "1/min", "min": 0, "max": 8000 } }
   ```

   `"input"` names the measurement on the axis (XCP-Ray marks the operating point); `"factor"`
   and `"offset"` scale the values, inside `"curve"` the axis. The firmware interpolates the
   curve itself (linearly between the points, as ECUs do). In XCP-Ray both the values and the
   axis points can be changed (the axis points stay ascending).

   **Characteristic maps** (z over x and y, edited as a heat map or table): a two-dimensional
   array `values[y][x]` and one ascending axis array for x and y each, e.g.
   `volatile float spark_x[8]; volatile float spark_y[6]; volatile float spark[6][8];` (y first,
   as in C: one row per y point) and in the list

   ```json
   { "name": "spark", "unit": "deg", "min": -10, "max": 50,
     "map": { "x": { "axis": "spark_x", "input": "rpm", "unit": "1/min" },
              "y": { "axis": "spark_y", "input": "load", "unit": "%" } } }
   ```

   The axis objects take the same keys as `"curve"`. Curves and maps may share an axis array.
   The firmware interpolates the map itself (bilinear). In XCP-Ray the values can be changed; the
   axis points of a map in the editor of a curve that shares its axis.

4. **Firmware:**

   ```cpp
   #include <x1xcp.h>

   static const x1xcp::Event kEvents[] = {{"10ms", 10}, {"100ms", 100, true}};

   void setup() {
       WiFi.begin(...);                      // as before: the project connects to the network
       WiFi.setSleep(false);                 // modem sleep delays packets by up to 100 ms
       x1xcp::Config xcp;
       xcp.events = kEvents;
       xcp.eventCount = 2;
       x1xcp::begin(xcp);                    // UDP port 5555
   }

   // in the task that computes the values of event 0, right after computing them:
   x1xcp::trigger(0, nowMs);                 // nowMs: e.g. the wake time of the cyclic task
   ```

   - Events with `autoTrigger` (third value `true`) are triggered by the XCP task every
     `cycleMs`; simple, but not synchronous with the application. `trigger()` samples in the
     calling task, exactly when the values are consistent.
   - Variables to measure or calibrate must be **global** (or `static` at file scope) and should
     be **`volatile`**, so the compiler neither keeps them in registers nor removes them.
     Parameters must not be `const` (those lie in flash and cannot be written).
   - `x1xcp::stats()` gives the connection state and counters (e.g. for a status LED).

5. **Build:** `pio run` writes the A2L. In XCP-Ray: Devices > Add ECU, the A2L, transport
   "XCP on UDP/IP"; the host comes from the A2L (`"hostName"` / `"address"` in the variable
   list) or is entered there.

## Technical data

- XCP on UDP/IP, port configurable (default 5555); MAX_CTO 255, MAX_DTO 1024; dynamic DAQ with
  up to 16 DAQ lists, 124 ODTs, 512 ODT entries; timestamps 4 bytes, 1 ms (FreeRTOS tick:
  `CONFIG_FREERTOS_HZ` must be 1000, the Arduino default); Intel byte order, address
  granularity BYTE.
- Memory: reads internal data RAM and flash-mapped constants, writes internal data RAM only
  (no PSRAM, no RTC RAM).
- Resources: XCP task (8 kB stack, core 1 by default, priority `configMAX_PRIORITIES - 2`), sender
  task (4 kB, core 0), about 40 kB of static RAM (send queue 12 x 1.4 kB, DAQ tables).
- DTOs are collected for `Config::flushMs` (default 10 ms) per datagram; a sender task on core 0
  calls `sendto()`, so the application is never blocked by the WLAN.

## Security

XCP has no authentication here (no SEED & KEY): every device in the network can read and
**write** the RAM of the ESP32 through port 5555. Use it only in trusted networks, and do not
ship it enabled in products without protection.

## License

MIT, see [LICENSE](LICENSE). Version 1.0.0.
