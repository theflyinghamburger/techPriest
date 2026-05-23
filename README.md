# Tech Priest Cosplay Props

Firmware repo for a Tech Priest cosplay prop set. Each prop has its own microcontroller.

## Projects

| Project | Path | Platform | Build system | Purpose |
|---------|------|----------|--------------|---------|
| plasmaPistol | `plasmaPistol/` | ESP32 DevKit V1 | PlatformIO | 9-LED prop gun + BLE |
| techPriest | `techPriest/` | ESP32 DevKit V1 | PlatformIO | LED googles, spotlight, joystick, servo arm |
| servoSkull | `servoSkull/` | ESP32 DevKit V1 | PlatformIO | OLED eye animation (SSD1306, I2C) |
| armDisplay | `armDisplay/` | Raspberry Pi Zero 2 W | Python | Touchscreen display |

Original Arduino IDE `.ino` files preserved in `legacy_code/`.

## Quick commands

Open each project folder in VS Code, then:

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

## State machine (plasmaPistol)

Four LED patterns driven by `gCurrentPatternNumber`:

| Index | Pattern | Trigger |
|-------|---------|---------|
| 0 | `idle` — green breathing | Default / after shooting |
| 1 | `charging` — blue + glitter | Button held |
| 2 | `overcharging` — blue→red blend | Button released after >2s hold |
| 3 | `shooting` — reverse LED kill | Button released after <2s hold |

## BLE (plasmaPistol)

- Advertises as `Plasma_Pistol`
- Service UUID: `09d2abe8-30ec-4519-86ff-ba0cbaf79160`
- Characteristic UUID: `102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107` (notify-only)
- Notifies a single uint8_t pattern index on state change
- Pairing PIN: `123456` (MITM bonding, encrypted GATT)

## Dependencies

- `fastled/FastLED@^3.10.3` — plasmaPistol
- `madhephaestus/ESP32Servo@^3.2.0` — techPriest
- `NeoPixelBus@^2.7.15` — techPriest
- `adafruit/Adafruit SSD1306@^2.5.14` — servoSkull
- `adafruit/Adafruit GFX Library@^1.11.9` — servoSkull

## Gotchas

- `startup()` uses `delay(200)` in a loop — blocks during boot animation
- `shooting()` is non-blocking — uses `millis()` with `shootingInterval` (20ms steps)
- Main loop frame rate is `1000 / FRAMES_PER_SECOND` ≈ 8ms (~120 fps), not 1 Hz
- `breathBrightness` in `idle()` clamps at 100–254, not the full 0–255 range
- `SecurityCallback::onPassKeyRequest()` returns `PASSKEY` (123456); the static passkey is also registered via `esp_ble_gap_set_security_param`
