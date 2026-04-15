# Darcy

Darcy is a Seeed XIAO ESP32-C3 based animatronic eye controller — a near-clone of Charles adapted for the C3 chip. It receives commands from a paired remote (Daryl) over ESP-NOW and drives two servos: one for left/right pan of the eyeball and one for eyelid open/close.

Board power comes over USB. Servo power is external with a common ground.

## Differences from Charles

- Uses a **Seeed XIAO ESP32-C3** instead of the ESP32-C6.
- Pan servo is on GPIO **4**; eyelid servo is on GPIO **5**.

## Remote control behavior (ESP-NOW)

Darcy listens for `RemotePacket` frames broadcast by Daryl. The packet carries an encoder position, encoder delta, a buttons bitmask, and an encoder-press flag.

### Encoder
- Rotating the encoder pans the eyeball left/right (D4 servo).
- Boot default is **FAST** gain: 13 deg per encoder step.
- Button 5 (bit 4) toggles gain between **FAST** (13 deg/step) and **SLOW** (5 deg/step).
- Pressing the encoder recenters the pan servo to 90 deg.

### Buttons (bits 0–5)
| Bit | Action |
|-----|--------|
| 0   | Snap pan to 0 deg |
| 1   | Toggle eyelid open/closed |
| 2   | Snap pan to 90 deg |
| 3   | Snap pan to 135 deg |
| 4   | Toggle encoder gain (FAST ↔ SLOW) |
| 5   | Reserved |

### Sleep signal
When a packet arrives with bits 6 or 7 set (out-of-band, outside the valid `0b00111111` mask), Darcy interprets it as Daryl entering deep sleep and enters **sleep pose**: pan centered at 90 deg, eyelid closed.

### Link loss
If no packet is received for 4 seconds, Darcy enters the same **sleep pose** (pan centered, eyelid closed).

### MAC filter
`DARYL_MAC` in `main.cpp` can be set to Daryl's MAC address to reject packets from other senders. All zeros disables the filter (accept any sender).

## Wiring
- Pan servo on GPIO 4
- Eyelid servo on GPIO 5
- Servo power KILL on GPIO 6 — HIGH cuts servo power via Adafruit #1400 KILL pin
- Servo power ON on GPIO 7 — pulse HIGH to simulate button press via S9013 transistor (1kΩ base resistor, emitter to GND pad, collector to button pad)
- Deep sleep wake on GPIO 3 — pull LOW to wake the board from deep sleep

## Power management

Servo power is controlled by an Adafruit Push-Button Power Switch (#1400) wired between the servo battery and the servo power rail.

### Servo power
- On boot, GPIO 6 is held LOW (KILL de-asserted) and GPIO 7 is pulsed HIGH for 250 ms to simulate a button press, ensuring the #1400 is switched on regardless of its prior state.
- When Darcy enters sleep pose (link loss or sleep signal from Daryl), it drives GPIO 6 HIGH after 300 ms to cut servo power.
- When a new packet arrives while servo power is off, GPIO 6 is released LOW, then GPIO 7 is pulsed HIGH for 250 ms to restore servo power.

### Board deep sleep
- 5 minutes after servo power is cut, the ESP32 enters deep sleep.
- If a packet arrives during that window, the deep sleep countdown is cancelled and normal operation resumes.
- The board wakes from deep sleep when GPIO 3 is pulled LOW and performs a full reboot, including the boot-time servo power-on pulse.

## Servos

Using Servo Motor Micro SG90

### Specs
| Property | Value |
|----------|-------|
| Drive | Analog |
| Rotation | 180° (±15°) |
| Voltage | 4.8–6 VDC (5V typical) |
| Current (idle) | 10 mA |
| Current (moving) | 100–250 mA |
| Current (stall) | 360 mA |
| Stall torque | 1.7 kg-cm |
| Speed | 0.12 s / 60° |
