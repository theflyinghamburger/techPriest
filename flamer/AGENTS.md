# flamer

ESP32 firmware for a Tech Priest flamer prop with smoke machine, LEDs, and BLE command control.

## Quick commands

| Action | Command |
|--------|---------|
| Build | `pio run` |
| Upload | `pio run --target upload` |
| Monitor serial | `pio run --target monitor` (115200 baud) |
| Clean build artifacts | `pio run --target clean` |

## Hardware wiring

- **LEDs**: 9x WS2812B on GPIO 5 (GRB color order)
- **Vape coil MOSFET**: GPIO 18 (PWM, ledc)
- **Motor pump MOSFET**: GPIO 19 (PWM, ledc)
- **Board**: ESP32 DevKit V1

## Code structure

- `src/flamer.cpp` — single-file application. All logic lives here.
- `include/` and `lib/` — empty stubs (PlatformIO defaults).
- `test/` — empty stub.

## State machine

Five states driven by `gState`:

| Index | State | Description |
|-------|-------|-------------|
| 0 | `IDLE` | LEDs off, mosfets off, waiting for BLE command |
| 1 | `RAMP_UP` | LEDs + mosfets ramp from 0→100% over 1000ms |
| 2 | `FLAMING` | Full fire effect on LEDs, mosfets at 100%, 30s auto-shutoff timer |
| 3 | `RAMP_DOWN` | LEDs + mosfets ramp from 100%→0 over 1000ms |
| 4 | `BOOT` | Startup animation (LEDs light up sequentially) |

## BLE

- Advertises as `TechPriest_Flamer`
- Service UUID: `09d2abe8-30ec-4519-86ff-ba0cbaf79160` (shared across props)
- Characteristic UUID: `102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107` (bidirectional: notify + write)
- **Receives**: `1` = turn ON, `0` = turn OFF
- **Notifies**: single uint8_t state index on state change
- Pairing PIN: `123456` (MITM bonding, encrypted GATT)

## LED fire effect

- Uses FastLED `fill_noise16` with Perlin noise mapped to fire palette
- Colors: black → dark red → orange → yellow → white hot
- Noise scale and speed increase during ramp-up for turbulence effect

## MOSFET control

- `ledc` PWM at 1000 Hz on GPIO 18 (coil) and GPIO 19 (motor)
- Both ramp together: 0→1023 over 1000ms (up), 1023→0 over 1000ms (down)

## Safety

- 30-second auto-shutoff if no OFF command received
- Ramp-down always completes fully before returning to IDLE

## Dependencies

- `fastled/FastLED@^3.10.3` — LED control
- ESP32 Arduino framework (BLE + ledc APIs built-in)

## Gotchas

- `ledc` channels must be unique — coil uses channel 0, motor uses channel 1
- Fire effect runs every frame (~120 fps) — noise seed advances each frame
- BLE write handler runs in callback context — only sets flag, main loop processes state transition
- Auto-shutoff resets on each new ON command
