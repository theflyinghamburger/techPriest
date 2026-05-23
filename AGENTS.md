# plasmaPistol

ESP32 firmware repo for a Tech Priest cosplay prop set. PlatformIO builds the plasma pistol; two other sketches are Arduino IDE–only.

## Projects

| Project | File | Build system | Purpose |
|---------|------|--------------|---------|
| plasmaPistol | `src/plasmaPistol.cpp` | PlatformIO | 9-LED prop gun + BLE |
| techPriest | `src/techPriest/techPriest.ino` | Arduino IDE only | Goggles, spotlight, joystick, servo arm |
| servoSkull | `src/servoSkull/servoSkull.ino` | Arduino IDE only | OLED eye animation (SSD1306, I2C) |

`platformio.ini` compiles `src/plasmaPistol.cpp` exclusively. The `.ino` files are not buildable from PlatformIO.

## Quick commands (plasmaPistol only)

| Action | Command |
|--------|---------|
| Build | `pio run` |
| Upload | `pio run --target upload` |
| Monitor serial | `pio run --target monitor` (115200 baud) |
| Clean | `pio run --target clean` |

## Hardware wiring (plasmaPistol)

- **LEDs**: 9x WS2812B on GPIO 5 (GRB color order)
- **Button**: GPIO 6 (input pullup, active-low)
- **Board**: ESP32 DevKit V1

## State machine

Four LED patterns driven by `gCurrentPatternNumber`:

| Index | Pattern | Trigger |
|-------|---------|---------|
| 0 | `idle` — green breathing | Default / after shooting |
| 1 | `charging` — blue + glitter | Button held |
| 2 | `overcharging` — blue→red blend | Button released after >2s hold |
| 3 | `shooting` — reverse LED kill | Button released after <2s hold |

## BLE

- Advertises as `Plasma_Pistol`
- Service UUID: `09d2abe8-30ec-4519-86ff-ba0cbaf79160`
- Characteristic UUID: `102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107` (notify-only)
- Notifies a single uint8_t pattern index on state change
- Pairing PIN: `123456` (MITM bonding, encrypted GATT, set via `esp_ble_gap_set_security_param`)

## Dependencies

- `fastled/FastLED@^3.10.3` — only external library for plasmaPistol
- ESP32 Arduino framework (BLE APIs built-in)

## Gotchas

- `startup()` uses `delay(200)` in a loop — blocks during boot animation
- `shooting()` is non-blocking — uses `millis()` with `shootingInterval` (20ms steps)
- Main loop frame rate is `1000 / FRAMES_PER_SECOND` ≈ 8ms (~120 fps), not 1 Hz
- `breathBrightness` in `idle()` clamps at 100–254, not the full 0–255 range
- `SecurityCallback::onPassKeyRequest()` returns `PASSKEY` (123456); the static passkey is also registered via `esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, ...)`
