# plasmaPistol — Code Review Issues

## High priority

1. ~~**`SecurityCallback::onPassKeyRequest()` returns `000000` instead of `PASSKEY`**~~ — **FIXED** (`src/plasmaPistol.cpp:121` now returns `PASSKEY`).
2. ~~**Potential null deref on auth failure**~~ — **FIXED** (`src/plasmaPistol.cpp:141` now guards with `if (pServer)`).

## Medium priority

3. ~~**`overcharging()` static `transitionStep` never resets**~~ — **FIXED** — moved to global `overchargingTransitionStep` (`src/plasmaPistol.cpp:38`), reset to `0` on entry (`src/plasmaPistol.cpp:264`).
4. ~~**`loop()` pattern tick ignores `FRAMES_PER_SECOND`**~~ — **FIXED** (`src/plasmaPistol.cpp:46` now uses `1000 / FRAMES_PER_SECOND` ≈ 8ms interval).
5. ~~**`shooting()` blocks event loop**~~ — **FIXED** — rewritten as non-blocking using `millis()` with `shootingInterval` (20ms steps) (`src/plasmaPistol.cpp:91-99`).
6. ~~**`onAuthenticationComplete()` restarts advertising on failure**~~ — **FIXED** — advertising only restarts on success; failure path removes peer device (`src/plasmaPistol.cpp:135-144`).
7. ~~**Redundant `notifyPatternChange()`**~~ — **FIXED** — single call at `src/plasmaPistol.cpp:273` after button-release branch.

## Low priority

8. ~~**Misleading comment**~~ — **FIXED** (`src/plasmaPistol.cpp:224` comment now reads "brief delay for BLE recovery").
9. ~~**Dead variables**~~ — **FIXED** — `buttonPressed` and `chargingComplete` removed.
10. ~~**Unused include**~~ — **FIXED** — `#include "esp_log.h"` removed (line 7).

---

**All 10 issues resolved. Code last verified: 2026-05-23.**
