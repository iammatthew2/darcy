# Darcy

Darcy is a Seeed XIAO ESP32-C3 based animatronic eye controller — a near-clone of Charles adapted for the C3 chip. It receives commands from a paired remote (Daryl) over ESP-NOW and drives two servos: one for left/right pan of the eyeball and one for eyelid open/close.

Board power comes over USB. Servo power is external with a common ground.

## Differences from Charles

- Uses a **Seeed XIAO ESP32-C3** instead of the ESP32-C6.
- Pan servo is on GPIO **4** (D4); eyelid servo is on GPIO **5** (D5).

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
- Pan servo on GPIO 4 (D4)
- Eyelid servo on GPIO 5 (D5)

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
