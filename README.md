# techPriest
ESP32 Arduino code for Tech Priest cosplay. 
Three ESP32 are used for modularity of parts.

## Projects

### techPriest.ino
- Goggles lighting
- Spotlight lighting
- Joystick sensors
- Servo Arm Motor control

### plasmaPistol.cpp
ESP32 firmware for a prop plasma pistol with WS2812B LEDs and BLE state reporting.

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

### servoSkull.ino
- Eye display animation
