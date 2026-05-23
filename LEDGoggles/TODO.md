# LEDGoggles — Code Review Issues

## Critical

- [x] **Missing `pinMode(LED_PIN, OUTPUT)`** — already present in setup (line 462)
- [x] **Unbraced `if` in `colorWipe` bounce logic** — fixed with braces
- [x] **`prevLEDCount` never resets on mode change** — added `prevLightingMode` tracking (line 217-221)
- [x] **Out-of-bounds pixel access in `colorWipe`** (line 247) — changed `>=` to `>`
- [x] **`analogRead()` called from hardware timer ISR** (line 404 → `updateJoystick` → `avg_flt` line 139) — moved to `loop()` with time gating (~60Hz)
- [x] **Race conditions on shared state between ISR and main/BLE contexts** — added `portMUX` critical sections

## Important

- [x] **`bleSecuritySetup()` called after advertising** — reordered before `BLEDevice::startAdvertising()` (line 357)
- [x] **Redundant `notifyModeChange()` in loop** — removed unconditional call, only called on BLE command (line 500)
- [x] **`spinningWheelsLED` only addresses pixels 0–22** — fixed to map `z` and `43-z` for all 44 pixels (lines 305-311)
- [x] **`SecurityCallback::onAuthenticationComplete` restarts advertising on success** (line 76) — removed
- [x] **No null check on `pCharacteristic` in `notifyModeChange()`** (line 300-301) — added null check
- [x] **`spinningWheelsLED` hardcodes pixel indices** (lines 276-289) — replaced with `PIXEL_COUNT` macro
- [x] **Unused ISR state: `milSecCount`, `secCount`, `status` cycle** (lines 389-399, 401-424) — simplified ISR to remove unused state
- [x] **`esp_log_level_set("*", ESP_LOG_DEBUG)` in `setup()`** (line 430) — changed to `ESP_LOG_WARN`
- [x] **`offset` not reset on mode change** (line 193) — reset on mode change
- [x] **`colorWipe` bounce never terminates** (lines 247-263) — fixed bounce logic
- [x] **`lightMilis` not reset on mode change** (line 193) — reset on mode change
- [x] **BLE re-advertising blocks `loop()` with `delay(500)`** (lines 458-466) — removed delay
- [x] **`removePeerDevice` with potentially stale connId** (line 80) — added connId != 0 check
- [x] **No watchdog feed** — added `esp_task_wdt_init(30, true)` + `esp_task_wdt_reset()` in loop

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
- [x] **`LED_PIN` (GPIO 2) naming confusion** (line 12) — renamed to use `LED_BUILTIN` (framework-provided)
- [x] **`hw_timer_t *Timer0_Cfg = NULL`** (line 127) — changed to `nullptr`
- [x] **Inconsistent bool comparisons** (lines 238, 246, 252, 258) — normalized to `!bounce`, `!ledBounce`, `ledBounce`
- [x] **`bleCommand >= 0` is dead code** (line 471) — removed redundant check
- [ ] **`Timer0_Cfg` never stopped or detached** — hardware timer runs forever; not a bug for embedded firmware but poor practice
