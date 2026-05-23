# plasmaPistol — Code Review Issues

## High priority

1. **`SecurityCallback::onPassKeyRequest()` returns `000000` instead of `PASSKEY`** — callback at `src/plasmaPistol.cpp:120` returns hard-coded `000000`; should return `PASSKEY` (123456) to match `esp_ble_gap_set_security_param` config.
2. **Potential null deref on auth failure** — `pServer->removePeerDevice()` at `src/plasmaPistol.cpp:140` has no null guard; `pServer` could be null if BLE init failed.

## Medium priority

3. **`overcharging()` static `transitionStep` never resets** — at `src/plasmaPistol.cpp:78`, `transitionStep` stays at 255 after first run; re-entering the pattern skips the blue→red transition entirely.
4. **`loop()` pattern tick ignores `FRAMES_PER_SECOND`** — at `src/plasmaPistol.cpp:45`, `interval = 1000` ms hardcodes 1 Hz; `FRAMES_PER_SECOND = 120` at line 19 is unused.
5. **`shooting()` blocks event loop** — at `src/plasmaPistol.cpp:92-96`, `delay(20)` loop blocks for ~180ms, freezing button reads and BLE re-advertise checks.
6. **`onAuthenticationComplete()` restarts advertising on failure** — at `src/plasmaPistol.cpp:142`, `BLEDevice::startAdvertising()` is called unconditionally, even when `cmpl.success` is false.
7. **Redundant `notifyPatternChange()`** — at `src/plasmaPistol.cpp:273` and `284`, the button-release path calls `notifyPatternChange()` twice in quick succession.

## Low priority

8. **Misleading comment** — at `src/plasmaPistol.cpp:222`, `delay(100)` is labeled "3 second delay for recovery."
9. **Dead variables** — `buttonPressed` (line 29) and `chargingComplete` (line 30) are set but never read.
10. **Unused include** — `#include "esp_log.h"` at line 7; no `ESP_LOG_*` macros are used anywhere.
