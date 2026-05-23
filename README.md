# Tech Priest Cosplay Props

Firmware and software for various props in a Tech Priest cosplay. Each prop runs on a separate microcontroller.

## Projects

| Project | File | Platform | Purpose |
|---------|------|----------|---------|
| techPriest | `src/techPriest/techPriest.ino` | ESP32 (Arduino IDE) | LED googles, spotlight, joystick sensors, servo arm control |
| servoSkull | `src/servoSkull/servoSkull.ino` | ESP32 (Arduino IDE) | OLED eye animation (SSD1306, I2C) |
| plasmaPistol | `src/plasmaPistol.cpp` | ESP32 (PlatformIO) | 9-LED prop gun + BLE state reporting |
| armDisplay | `src/armDisplay/armDisplay.py` | Raspberry Pi Zero 2 W | Touchscreen display |

## Build Systems

- **plasmaPistol**: PlatformIO (`pio run`)
- **techPriest, servoSkull**: Arduino IDE only
- **armDisplay**: Python script, run directly on Pi

`platformio.ini` compiles `src/plasmaPistol.cpp` exclusively. The `.ino` files are not buildable from PlatformIO.

## Plasma Pistol Details

**Hardware:**
- Board: ESP32 DevKit V1
- LEDs: 9x WS2812B on GPIO 5 (GRB color order)
- Button: GPIO 6 (input pullup, active-low)

**LED Patterns:**
- `idle` — green breathing (default state)
- `charging` — blue + glitter (button held)
- `overcharging` — blue→red blend (button released after >2s hold)
- `shooting` — reverse LED kill, non-blocking (button released after <2s hold)

**BLE:**
- Advertises as `Plasma_Pistol`
- Service UUID: `09d2abe8-30ec-4519-86ff-ba0cbaf79160`
- Characteristic UUID: `102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107` (notify-only)
- Notifies a single uint8_t pattern index on state change
- Pairing PIN: `123456` (MITM bonding, encrypted GATT)

**Dependencies:**
- `fastled/FastLED@^3.10.3`
- ESP32 Arduino framework (BLE APIs built-in)
