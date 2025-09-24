# ESP32-H2 LED Status Indicators

This project uses **two LEDs** on the ESP32-H2 dev board:

1. **RGB LED (WS2812) on GPIO8** – fully controllable, used for main system status.
2. **Green LED (GPIO, pin TBD)** – simple on/off LED, optional secondary indicator.

---

## RGB LED (GPIO8)

The onboard RGB LED shows the **system state**:

| State           | LED Color / Pattern                       |
| --------------- | ----------------------------------------- |
| Boot            | OFF                                       |
| GNSS Searching  | Yellow, slow blink (1 Hz)                 |
| GNSS 2D Fix     | Green, fast blink (4 Hz)                  |
| GNSS 3D Fix     | Green, slow blink (1 Hz)                  |
| BLE Connected   | Blue, short flash every 5 s               |
| Error           | Red, fast blink (8 Hz)                    |
  ---

## Green LED (GPIO? – check your board)

This LED is optional and depends on your specific ESP32-H2 board:
- On some boards it’s **wired to a UART chip** → flashes during upload/reset only (not controllable).
- On others it’s **wired to a GPIO** (e.g. GPIO2, GPIO9) → can be controlled in code.

If controllable, you can use it as a **secondary indicator**, for example:
- **Blink on BLE activity** (data TX/RX).
- **Heartbeat** (periodic short flash when the device is healthy).

---

## Priority Rules

Since there is only one RGB LED:
1. **Error (red)** always overrides other states.  
2. **Boot (purple)** shows until init finishes.  
3. **GNSS state (yellow/blue/green)** is the main status.  
4. **BLE events** may temporarily override (cyan blink) but return to GNSS state.  

---

## Quick Reference

- 🔴 **Red blink** = Error
- 🟣 **Purple breathing** = Booting
- 🟡 **Yellow blink** = GNSS searching
- 🔵 **Blue steady** = 2D GNSS fix
- 🟢 **Green steady** = 3D GNSS fix
- 🟦 **Cyan blink** = BLE advertising
- 🟦 **Cyan steady** = BLE connected

---


