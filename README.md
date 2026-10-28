# Mini ADAS: Forward Collision Warning on ROS2

A small-scale forward collision warning (FCW) system built on **ROS2 Humble**, running on a Raspberry Pi 4. It fuses a **camera** (pedestrian detection) with an **ultrasonic range sensor** (distance) and drives a physical **LED + buzzer** alert through an Arduino, following the same sense → perceive → decide → act loop used in automotive ADAS.

![demo](docs/demo.gif)

## Why two sensors

| Sensor | Good at | Weak at |
|---|---|---|
| Camera | *What* is ahead (a pedestrian vs. a box) | Measuring distance |
| Ultrasonic | *How far* away it is | Telling what the object is |

Fusing them lets the system react **earlier for a pedestrian** than for a generic obstacle at the same distance, which neither sensor can do alone.

## Architecture

```
 ┌──────────────┐  /camera/image_raw   ┌────────────────┐  /perception/pedestrian_in_path
 │ camera_node  │ ───────────────────▶ │ detection_node │ ──────────────────────┐
 └──────────────┘                      └────────────────┘                       ▼
                                          │ /perception/annotated/compressed ┌─────────────┐
                                          ▼  (Foxglove)                      │ fusion_node │
 ┌──────────┐  USB serial  ┌─────────────────────┐  /ultrasonic/range        │             │
 │ Arduino  │ ───D:cm────▶ │ arduino_bridge_node │ ─────────────────────────▶│             │
 │ HC-SR04  │ ◀──W:level── │                     │ ◀──── /adas/warning ───── │             │
 │ LED/buzz │              └─────────────────────┘   (adas_interfaces/        └─────────────┘
 └──────────┘                                          CollisionWarning)
```

| Node | Role |
|---|---|
| `camera_node` | Publishes 320×240 webcam frames (`sensor_msgs/Image`) |
| `detection_node` | OpenCV HOG pedestrian detector, throttled to 5 Hz; flags detections inside a central "forward zone" and publishes an annotated image |
| `arduino_bridge_node` | Owns the serial port in both directions: publishes `sensor_msgs/Range`, sends warning levels back to the Arduino |
| `fusion_node` | Combines both inputs at 20 Hz into a custom `CollisionWarning` message |

`static_transform_publisher` nodes define where each sensor sits on the vehicle body (`base_link → camera_link`, `base_link → ultrasonic_link`).

## Decision logic

| Condition | Level |
|---|---|
| Ultrasonic data stale (> 0.5 s) | **CAUTION** (fail-safe: never SAFE without range data) |
| Distance ≤ 0.30 m | **BRAKE** |
| Distance ≤ 0.80 m **and** pedestrian in path | **BRAKE** |
| Distance ≤ 0.80 m | **CAUTION** |
| Pedestrian in path, beyond range | **CAUTION** |
| Otherwise | **SAFE** |

Warnings **escalate immediately** but **de-escalate only after 5 consistent cycles** (hysteresis), so the buzzer doesn't flicker when a reading sits on a threshold. The Arduino also falls back to CAUTION on its own if it hears nothing from the Pi for 1 s.

## Design notes

- **The HC-SR04 is on the Arduino, not the Pi.** Its echo pin outputs 5 V, which would damage the Pi's 3.3 V GPIO. The Arduino also handles the microsecond-level echo timing that's unreliable under Linux.
- **One node owns the serial port.** A serial device can only be opened by one process, so sensor input and alert output share `arduino_bridge_node` rather than being split.
- **HOG is throttled.** It's CPU-heavy on a Pi 4, so detection runs at 5 Hz on 320×240 frames while fusion runs at 20 Hz using the latest result.
- **Custom message.** `CollisionWarning` carries the level, distance, per-sensor health flags and a reason string, so the decision is easy to inspect in Foxglove.

## Hardware

- Raspberry Pi 4B, Ubuntu Server 22.04, ROS2 Humble
- Arduino Uno (or similar) over USB
- HC-SR04 ultrasonic sensor, LED + 220 Ω resistor, buzzer
- USB webcam

**Wiring:** HC-SR04 VCC→5V, GND→GND, Trig→D9, Echo→D10 · LED→D5 · Buzzer→D6

## Build and run

```bash
# Flash arduino/ultrasonic_bridge/ultrasonic_bridge.ino with the Arduino IDE first
# The repo root is itself a colcon workspace (packages live in src/)
git clone https://github.com/<your-username>/mini-adas-ros2.git ~/mini-adas-ros2
cd ~/mini-adas-ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch mini_adas mini_adas.launch.py            # add port:=/dev/ttyUSB0 if needed
```

**Foxglove:** open Foxglove Studio on your laptop → *Open connection* → `ws://<pi-ip>:8765`. Useful panels: Image (`/perception/annotated/compressed`), Plot (`/ultrasonic/range.range`, `/adas/warning.level`), Raw Messages (`/adas/warning`).

**Quick checks:**
```bash
ros2 topic hz /ultrasonic/range     # ~20 Hz
ros2 topic echo /adas/warning
```

## Next steps

- Replace HOG with a lightweight neural detector (e.g. MobileNet-SSD) for better accuracy
- Estimate time-to-collision from the rate of change of distance, not just distance
- Replace the ultrasonic with a mmWave radar for longer range and relative velocity
