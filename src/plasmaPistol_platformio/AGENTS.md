# plasmaPistol

ESP32 firmware for a prop plasma pistol with WS2812B LEDs and BLE state reporting.

## Quick commands

| Action | Command |
|--------|---------|
| Build | `pio run` |
| Upload | `pio run --target upload` |
| Monitor serial | `pio run --target monitor` (115200 baud) |
| Clean build artifacts | `pio run --target clean` |

## Hardware wiring

- **LEDs**: 9x WS2812B on GPIO 5 (GRB color order)
- **Button**: GPIO 6 (input pullup, active-low)
- **Board**: ESP32 DevKit V1

## Code structure

- `src/plasmaPistol.cpp` — single-file application. All logic lives here.
- `include/` and `lib/` — empty stubs (PlatformIO defaults). No custom headers or local libs.
- `test/` — empty stub. No unit tests.

## State machine

Four LED patterns driven by a single `gCurrentPatternNumber` index:

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
- Pairing PIN: `123456` (MITM bonding, encrypted GATT)

## Dependencies

- `fastled/FastLED@^3.10.3` — only external library
- ESP32 Arduino framework (BLE APIs are built-in)

## Gotchas

- `shooting()` uses `delay()` in a loop — it blocks the event loop during the animation
- The `loop()` fires patterns at 1 Hz (`interval = 1000` ms), not at `FRAMES_PER_SECOND`
- `SecurityCallback::onPassKeyRequest()` returns `000000` regardless of `PASSKEY` — the static passkey is set via `esp_ble_gap_set_security_param` instead
- `breathBrightness` in `idle()` clamps at 100–254, not the full 0–255 range
