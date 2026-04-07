# Proposed Fixes for STM32Duino Firmware Libraries

This document summarizes concrete bugs found in the firmware libraries in this directory and proposes fixes for each one.

## 1. APRS packet builder can overflow fixed-size buffers

- Problem: `APRS_Bits.cpp` copies `path` into a 50-byte stack buffer with `strncpy` and then appends packet data into fixed-size `frame` and `bits` buffers without checking whether the destination still has space.
- Impact: Long callsigns, digipeater paths, or APRS info payloads can corrupt memory and produce invalid packets or hard faults.
- Proposed fix:
  - Validate `source`, `dest`, `path`, and `info` lengths before encoding.
  - Force `path_copy` to be null-terminated after `strncpy`.
  - Track remaining capacity while building the AX.25 frame.
  - Abort or truncate cleanly if `frame_len > APRS_MAX_FRAME` or `bit_len > APRS_MAX_BITS`.
  - Consider changing `sendPacket()` to return `bool` so callers can detect failure.

## 2. `pyro` drives invalid pins on unsupported STM32 variants

- Problem: In `pyro.cpp`, when `PE10` is not defined, all pin arrays are left as zero-initialized globals.
- Impact: On STM32F0/C0/H0 boards that do not expose those exact pins, the library configures and writes pin `0` for every pyro channel, which is incorrect and potentially dangerous.
- Proposed fix:
  - Replace the `#else` branch with a hard compile-time error using `#error` for unsupported boards.
  - Or make pins constructor parameters so the board mapping is explicit per target.
  - If a fallback must exist, add a `validPinMap` flag and refuse to arm or fire unless the map is configured.

## 3. `pyro` state arrays are never initialized

- Problem: `_armed`, `_fired`, `_firedTimes`, and `_pyroStatus` are never initialized in the constructor or `begin()`.
- Impact: Random startup memory can make channels appear armed, fired, failed, or already timed out.
- Proposed fix:
  - Zero all state in the constructor or `begin()`.
  - Initialize every channel status to `PYRO_UNCONNECTED` or a defined startup state.
  - Explicitly drive all fire pins low during `begin()`.

## 4. `ADXL357` integrates velocity from uninitialized timing/state

- Problem: `ADXL357.cpp` uses `_lastUpdate` and `_integratedVelo` before setting them to known values.
- Impact: The first accelerometer sample can create a huge bogus integrated velocity.
- Proposed fix:
  - Initialize `_lastUpdate = micros()` during `setup()` or on the first successful sample.
  - Initialize `_integratedVelo = 0.0f` and `_verticalAccelMinusGravity = 0.0f` in the constructor.
  - Skip integration on the first valid sample after boot.

## 5. `gyro` attitude integration starts from uninitialized state

- Problem: `gyro.cpp` integrates roll, pitch, and yaw using `_lastUpdate`, `_roll`, `_pitch`, and `_yaw` without initialization.
- Impact: The first update can jump attitude by an arbitrary amount and poison subsequent control logic.
- Proposed fix:
  - Initialize all attitude members to zero in the constructor.
  - Set `_lastUpdate = micros()` in `begin()` or on the first `update()` call.
  - Skip the first delta-time integration step until a valid previous timestamp exists.

## 6. `baro` filtered altitude buffer and offsets are uninitialized

- Problem: `baro` uses `_filteredAltSamples`, `_filteredAlt`, `_maxAlt`, and `_heightOffset` before initialization.
- Impact: Startup altitude, filtered altitude, and max altitude can begin with garbage values.
- Proposed fix:
  - Zero the sample buffer and all altitude state in `begin()` or the constructor.
  - On first run, seed the moving-average buffer with the first measured altitude rather than zeros or garbage.
  - Only update `_maxAlt` after the filter has been initialized.

## 7. `GPS` parser state is uninitialized at startup

- Problem: `_headerValid`, `_maxAlt`, and `_heightOffset` are not initialized.
- Impact: The parser can skip sync detection on boot and altitude outputs can be undefined until `zeroAlt()` is called.
- Proposed fix:
  - Initialize `_headerValid = false`, `_maxAlt = 0.0f`, and `_heightOffset = 0.0f` in the constructor.
  - Zero `_pkt` and `_buf` to avoid stale data appearing as valid telemetry.
  - Optionally add a `hasFix` or `hasPacket` flag so callers know whether data is valid yet.

## 8. `power` exposes signed telemetry as unsigned values

- Problem: `power.h` declares several getters as `uint16_t`, but the packet fields they return are `int16_t`.
- Impact: Negative currents or signed rail readings wrap into large positive values.
- Proposed fix:
  - Change the affected getter return types to `int16_t`.
  - Audit downstream callers for assumptions about unsigned values.
  - If some channels are physically unsigned, clamp or convert at the interface with explicit documentation.

## 9. `BQ76922::cellConfig()` can write an undefined register value

- Problem: `cellConfig()` only assigns `config` when `numCells == 3`; other inputs leave it uninitialized.
- Impact: Passing any unsupported cell count writes an unpredictable value to configuration RAM.
- Proposed fix:
  - Initialize `config` to a safe default.
  - Validate `numCells` explicitly and return `false` for unsupported values.
  - Prefer a `switch` covering each supported cell count.

## 10. `BQ76922::statusReadout()` decodes mode bits incorrectly

- Problem: Expressions like `status & (3 << 8) == 0` rely on operator precedence and are parsed incorrectly.
- Impact: The reported BQ76922 access mode can be wrong even if the raw status register is correct.
- Proposed fix:
  - Extract the field first, for example `uint16_t secMode = (status >> 8) & 0x3;`.
  - Compare `secMode` against `0`, `1`, `2`, and `3` explicitly.
  - Apply the same style to any similar status-field decoding in the file.

## 11. `airbrakes::inverse2x2Matrix()` computes the inverse incorrectly

- Problem: The function writes `Ainv[1][0]` twice and never assigns `Ainv[0][0]`.
- Impact: Any caller using this helper receives an invalid matrix inverse.
- Proposed fix:
  - Replace the body with the standard 2x2 inverse formula:
    - `Ainv[0][0] =  f * A[1][1]`
    - `Ainv[0][1] = -f * A[0][1]`
    - `Ainv[1][0] = -f * A[1][0]`
    - `Ainv[1][1] =  f * A[0][0]`
  - Add a focused unit test if this module has a test harness later.

## 12. General startup-state cleanup across sensor classes

- Problem: Several classes rely on power-on memory contents instead of explicit initialization.
- Impact: Behavior differs between resets, toolchains, and optimization levels, which is especially risky on embedded targets.
- Proposed fix:
  - Initialize every member in constructors or `begin()` methods.
  - For timing-based integrators, treat the first sample as a priming step.
  - For telemetry parsers and moving averages, add explicit `valid` or `initialized` flags.

## Recommended Implementation Order

1. Fix `pyro` board mapping and initialization first because it can affect safety-critical outputs.
2. Fix `ADXL357`, `gyro`, `baro`, and `GPS` initialization next because they directly affect state estimation.
3. Fix `APRS_Bits` buffer bounds after that because it is a memory-safety issue.
4. Fix `BQ76922` status decoding and `cellConfig()` validation next.
5. Fix `power` signedness mismatches and the `airbrakes` helper afterward.
