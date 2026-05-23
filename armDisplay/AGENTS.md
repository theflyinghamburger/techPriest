# armDisplay — Tech Priest Arm Display Firmware

## Overview
Python tkinter CRT-styled GUI for Raspberry Pi Zero 2 W touchscreen. Four mode-select buttons with scanline, vignette, flicker, and noise effects. Intended to communicate with other props via BLE.

## Hardware
- **Board**: Raspberry Pi Zero 2 W
- **Display**: Pi touchscreen (480x320 native)
- **Language**: Python 3, tkinter + Pillow

## Dependencies
- `Pillow` (PIL fork) — image loading, drawing, resizing

## Entry point
- `armDisplay.py` — run with `python3 armDisplay.py`

## UI layout
- Fullscreen tkinter window (480x320 logical, scaled to fullscreen)
- Background image: `background.jpeg` (loaded, resized to 480x320)
- 4 buttons in lower portion: `COMBAT`, `EXPLORE`, `ENGAGE`, `RESET`
- CRT effects: scrolling scanline, vignette overlay, screen flicker, noise overlay

## Effects
| Effect | Method | Interval |
|--------|--------|----------|
| Scanline | `scroll_line_effect()` | 30ms |
| Screen flicker | `screen_flicker()` | 200ms |
| Noise overlay | `noise_overlay()` | 100ms |
| Vignette | `apply_vignette()` | Once at init |

## BLE (PR #14)
- **Library**: `bleak` (Python BLE client, graceful fallback if missing)
- **BLE_Controller class**: `discover()`, `connect()`, `disconnect()`, `send_command()`
- **Discovery**: auto-scans for `Plasma_Pistol` on startup (10s timeout)
- **Status label**: top-right indicator (red = disconnected, lime = connected)
- **Event loop**: asyncio runs in background daemon thread
- **Shutdown**: `Ctrl+Q` triggers BLE disconnect before exit
- **Target**: service `09d2abe8-30ec-4519-86ff-ba0cbaf79160`, char `102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107`
- **Sends**: single uint8_t button index (0-3) via `write_gatt_char`
- **Note**: receiving props need writable characteristic (plasmaPistol currently notify-only)

## Gotchas (remaining)
- Button labels are generic; don't match Tech Priest theme
- Receiving props need writable BLE characteristic (plasmaPistol currently notify-only)
- No reconnection logic if BLE drops mid-session
- `bleak` requires BlueZ 5.43+ — verify Pi OS version

## Fixed in PR #13
- ~~`noise_overlay()` memory leak~~ — now tracks & deletes canvas item
- ~~`screen_flicker()` photosafety~~ — reduced to subtle `#0a0a0a` variation
- ~~Vignette blocks main thread~~ — stepped loop (60 iterations vs 360)
- ~~Missing `background.jpeg` crash~~ — try/except with logger warning
- ~~No graceful shutdown~~ — `Ctrl+Q` bound to `shutdown()`
