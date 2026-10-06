# AGENTS.md - Project Context & Operational Rules for AI Agents

This file gives non-negotiable architectural context, coding standards and verification workflow
for agents working on **tuf2000**, a standalone PlatformIO/Arduino library (ESP32, pioarduino) for
the TUF-2000M ultrasonic flow meter over Modbus RTU/RS-485, built on eModbus. It is a published
library: other people's firmware depends on it, so API stability and fail-safe decoding matter more
than feature velocity. Read it fully before any task.

---

## 1. Project Overview & Architecture

### Layout

```
src/                 Tuf2000Protocol.*  pure C++ logic (no Arduino/eModbus/FreeRTOS)
                     Tuf2000FlowMeter.* eModbus-backed driver (Arduino + FreeRTOS)
                     Tuf2000.h          umbrella header
examples/            Arduino/PlatformIO example sketches (must compile in CI)
test/test_tuf2000/   host-run Unity tests for everything in Tuf2000Protocol.*
library.json         PlatformIO manifest (source of truth for version + dependencies)
library.properties   Arduino Library Manager manifest (version must match library.json)
platformio.ini       dev/CI environments only - consumers never see it
```

### Design rules

* **Protocol vs. driver split:** decoding, validation, poll sequencing and staleness logic live in
  `Tuf2000Protocol.*` and must not include `Arduino.h`, eModbus or FreeRTOS headers. That is what
  makes `pio test -e native` possible. `Tuf2000FlowMeter.*` only glues that logic to eModbus.
* **No application coupling:** the library must contain **no pin numbers, no board names, no
  project logger, no MQTT/Home Assistant/irrigation concepts**. Everything board- or
  application-specific arrives through `Tuf2000FlowMeter::Config` or `setLogger()`.
* **No globals, no hidden singletons:** the caller owns the `Tuf2000FlowMeter` instance. The library
  never constructs hardware objects on first use.
* **Namespace:** all public symbols live in `namespace tuf2000`. Do not leak names (including via
  `using namespace` in headers). eModbus itself leaks `using namespace Modbus;`; do not add more.
* **Stable public API:** the library is semver-versioned. Renaming/removing a public member or
  changing a `Config` default is a breaking change (see section 5). Prefer adding fields with
  safe defaults.
* **Non-blocking:** `readAsync()`/`takeReading()` must never block. The only blocking call is the
  documented `read()` convenience, which library code and the examples' `loop()` must not use.

---

## 2. Non-Negotiable Coding Standards

* **Thread safety:** eModbus callbacks run on its worker task; `readAsync()`/`takeReading()` run on
  the caller's task. Cross-task data goes through the depth-1 `xQueueOverwrite` mailbox or a
  `volatile` single-word flag. Do not add shared mutable state without documenting who writes and
  who reads it.
* **Logger callback:** it may be called from the worker task. Format messages into a small stack
  buffer with `snprintf`; no `String`, no heap allocation, no blocking in library code paths. The
  library never calls `Serial` directly (the example sketch may).
* **Avoid heap churn:** no `String` concatenation, no per-poll allocation. Allocate once in
  `begin()`.
* **Prefer `constexpr`/`static const` over `#define`.**
* **Don't repeat yourself:** before adding a function, look for an existing one to reuse or extend.
  Widen an existing function if it has only 1-2 callers; otherwise add a broader one and have the
  narrow one delegate to it.
* **Formatting:** run `clang-format` (LLVM-based, ~100 columns) on modified files.
* **Static analysis:** `pio check` must introduce no new warnings.
* **Generated artifacts:** never edit anything under `.pio/`.

---

## 3. Error Handling

* **No silent swallowing:** every eModbus return code (`addRequest`, error callbacks) is recorded
  (`lastErrorCode()`) and surfaced through `Reading::valid`/`Reading::failure`, and logged when a
  logger is set.
* **Explicit failure over default data:** a reply that cannot be trusted - wrong length, NaN or
  infinity, an implausible totalizer jump - rejects the **whole poll** (`valid == false` with a
  `Tuf2000Failure` reason). Never clamp, substitute zero, or "repair" a value. A truncated reply
  must never read back as zero flow.
* **Reject bad configuration:** `begin()` returns `false` for an unusable `Config` (unset pins, zero
  poll interval) instead of guessing.
* The library only reports. What a failure means for the application (alerts, valve shut-off, LED
  patterns) is the application's job.

---

## 4. Operational Steps & Verification Loop

Run PlatformIO from **PowerShell or cmd, not Git Bash**: under MSys the pioarduino platform cannot
install its tools.

### Step 1: Baseline
1. Pull the latest changes.
2. `pio test -e native` passes.
3. The example builds for ESP32 (see Step 4.3).

### Step 2: Implement
1. Pure logic goes in `Tuf2000Protocol.*`; eModbus/Arduino glue in `Tuf2000FlowMeter.*`.
2. Add or update a native test for every new or changed function in `Tuf2000Protocol.*`; remove
   tests of removed code.
3. Changes confined to `Tuf2000FlowMeter.*` cannot be host-tested; note the manual hardware check
   you did (Step 5).
4. Update `README.md` and the example when public API or behavior changes, and add a line under
   `[Unreleased]` in `CHANGELOG.md`.

### Step 3: Clean up
* No stray `Serial.print` in `src/` (use the logger callback), no commented-out code, no unused
  includes, no project-specific names or pin numbers.

### Step 4: Verify
1. `pio test -e native` - all pass.
2. `pio check -e esp32dev` - no new warnings.
3. Compile `examples/basic` for the ESP32 environment on the pioarduino platform; no new warnings
   from this library's own code.
4. If `library.json` changed: `pio pkg pack` and inspect the tarball contents.

### Step 5: Hardware smoke test
A clean build and passing native tests do **not** prove Modbus timing or word order are right.
Before a release, run the example against a real TUF-2000M over RS-485 and confirm: valid readings
arrive at the poll interval, flow rate and totalizer match the meter's display, and unplugging the
bus produces `valid == false` readings with `kModbusError`, with recovery once reconnected.

---

## 5. Versioning & Release

* Semantic versioning. Bump the **patch** version for fixes and the **minor** version for
  backward-compatible features. Bump **major** (or the middle component while `0.x`) only when the
  developer asks or on a breaking API change.
* Keep the version identical in `library.json`, `library.properties` and `CHANGELOG.md`; tag
  releases `vX.Y.Z`.
* Publishing to the PlatformIO registry is public and versions cannot be overwritten: never run
  `pio pkg publish` or push tags without an explicit request.
* The eModbus dependency in `library.json` stays pinned to a tested version. Changing it requires
  re-running Steps 4 and 5.
* Do not add the TUF-2000M manual PDF or other third-party copyrighted material to the repo; link
  to it. Anything copied in must be compatible with the MIT license.
