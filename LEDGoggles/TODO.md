# LEDGoggles — Code Review Issues

## Critical

- [x] **Missing `pinMode(LED_PIN, OUTPUT)`** — already present in setup (line 462)
- [x] **Unbraced `if` in `colorWipe` bounce logic** — fixed with braces
- [x] **`prevLEDCount` never resets on mode change** — added `prevLightingMode` tracking (line 217-221)
- [ ] **Out-of-bounds pixel access in `colorWipe`** (line 247) — `PixelCount() >= prevLEDCount` allows `SetPixelColor(44, ...)` on a 44-pixel strip (valid: 0–43); change `>=` to `>`
- [ ] **`analogRead()` called from hardware timer ISR** (line 404 → `updateJoystick` → `avg_flt` line 139) — blocking ADC call in 2ms ISR risks missed ticks, queued ISR invocations, and ADC2/WiFi deadlock; move to `loop()` with time gating
- [ ] **Race conditions on shared state between ISR and main/BLE contexts** — `pos`, `mode`, `bleCommand`, `bleCommandPending`, `xfilteredOutput`, `yfilteredOutput` written in ISR and read in `loop()`/BLE callbacks with no synchronization; `pos++` (line 374) is non-atomic; use critical sections or `atomic` operations

## Important

- [x] **`bleSecuritySetup()` called after advertising** — reordered before `BLEDevice::startAdvertising()` (line 357)
- [x] **Redundant `notifyModeChange()` in loop** — removed unconditional call, only called on BLE command (line 500)
- [x] **`spinningWheelsLED` only addresses pixels 0–22** — fixed to map `z` and `43-z` for all 44 pixels (lines 305-311)
- [ ] **`SecurityCallback::onAuthenticationComplete` restarts advertising on success** (line 76) — `BLEDevice::startAdvertising()` after successful connection interferes with active connection; remove
- [ ] **No null check on `pCharacteristic` in `notifyModeChange()`** (line 300-301) — if `bleSetup()` fails partway, `setValue()`/`notify()` hard-fault on nullptr
- [ ] **`spinningWheelsLED` hardcodes pixel indices** (lines 276-289) — `z < 22` and `43 - z` are magic numbers; use `PIXEL_COUNT / 2` and `PIXEL_COUNT - 1 - z` for portability
- [ ] **Unused ISR state: `milSecCount`, `secCount`, `status` cycle** (lines 389-399, 401-424) — incremented every 2ms but never read; 6 of 8 switch cases are empty; wastes ~16% of ISR execution time
- [ ] **`esp_log_level_set("*", ESP_LOG_DEBUG)` in `setup()`** (line 430) — floods UART at 115200 baud, degrades performance significantly; change to `ESP_LOG_WARN` for production
- [ ] **`offset` not reset on mode change** (line 193) — switching away from `spinningWheelsLED` and back retains stale `offset`, causing visual discontinuity
- [ ] **`colorWipe` bounce never terminates** (lines 247-263) — when `prevLEDCount` reaches 0 with `ledBounce == true`, the else branch toggles `ledBounce` to `false`, then the next cycle re-enters the "light up" branch; infinite bounce loop
- [ ] **`lightMilis` not reset on mode change** (line 193) — first frame after mode change may be skipped or delayed depending on timing relative to last `lightMilis` update
- [ ] **BLE re-advertising blocks `loop()` with `delay(500)`** (lines 458-466) — blocks lighting/servo for 500ms; `deviceConnected` could flip back to true during delay, causing `startAdvertising()` while connected
- [ ] **`removePeerDevice` with potentially stale connId** (line 80) — `getConnId()` may return invalid ID after auth callback if client disconnected during handshake; can crash BLE stack
- [ ] **No watchdog feed** — `delay(500)` in BLE re-advertising + debug UART flood can trip ESP32 task watchdog under load

## Efficiency (planned)

- [ ] **`strip1.Show()` called per-pixel in `colorWipe`** (lines 242, 249, 254, 260) — each `Show()` triggers full 44-pixel DMA/I2S transfer (~30µs); needed for wipe animation effect but wasteful for mode 0
- [ ] **`colorWipe` bounce does redundant work for mode 0** — mode 0 calls `colorWipe(0,0,0,50,true)` which sweeps all 44 pixels on (black) then off (black), issuing 88 `Show()` calls; replace with single `clearLED()` call
- [ ] **`servoMove` fires every 15ms but called every `loop()` iteration (~8ms)** — minor overhead from `millis()` check each cycle; acceptable but could be optimized

## Cleanup (planned)

- [x] **Unused variables** — removed `oldState`, `lightCasePrint`, `swfilteredOutput`, `timer`, `LED` define, `servoBounce`
- [x] **Dead/commented code** — removed `servoSweep()` function, commented Serial.print in `printSensorInfo()`, `//avg_flt(swADC, ...)`, `//ESP32PWM::allocateTimer(0)`
- [x] **`prevMilis` array over-allocated** — reduced from 8 to 2 elements
- [x] **Inconsistent servo bounds** — `armPos` uses 0–180, matches `myservo.attach(servoPin, 500, 3500)` (line 473)
- [ ] **Missing `swReadings`/`swSum`** — switch button ADC filtering (`swADC`/line 18) has no arrays; add `u_int16_t swReadings[FILTER_ORDER]` and `u_int16_t swSum = 0` if the switch feature is needed
- [ ] **`LED_PIN` (GPIO 2) naming confusion** (line 12) — `LED_PIN` is the built-in LED, separate from `PIXEL_PIN` (GPIO 15, NeoPixels); rename to `BUILTIN_LED` for clarity
- [ ] **`hw_timer_t *Timer0_Cfg = NULL`** (line 127) — should use `nullptr` for C++ code
- [ ] **Inconsistent bool comparisons** (lines 238, 246, 252, 258) — mix of `bounce == false`, `ledBounce == false`, `ledBounce == 1`; use `!bounce`, `!ledBounce`, `ledBounce`
- [ ] **`bleCommand >= 0` is dead code** (line 471) — `bleCommand` is `uint8_t`, so `>= 0` is always true
- [ ] **`Timer0_Cfg` never stopped or detached** — hardware timer runs forever; not a bug for embedded firmware but poor practice
