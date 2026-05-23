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

## BLE
Not yet implemented. `button_action()` prints to console; intended to send commands to other props (plasmaPistol, LEDGoggles, servoSkull) over BLE.

## Gotchas
- `noise_overlay()` creates a new `PhotoImage` + canvas item every 100ms without deleting the old one — memory leak
- `screen_flicker()` can flash canvas bg to bright green — photosensitivity concern
- Vignette draws 360 ellipses at startup — blocks main thread briefly
- No error handling if `background.jpeg` is missing
- Button labels are generic; don't match Tech Priest theme
- No graceful shutdown handler
