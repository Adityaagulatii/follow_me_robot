# Follow-Me Robot

A real-time autonomous person-following robot that locks onto a target using computer vision, tracks them through occlusions and room transitions, and drives a physical wheeled robot over Bluetooth — running entirely on a laptop with no cloud dependency.

---

## What it does

Point the camera at yourself, press lock, and the robot follows you around the room. If you walk behind furniture, it searches. If you leave the room, it pursues. If it loses you completely, it stops and waits.

https://github.com/Adityaagulatii/follow_me_robot/raw/main/demo.gif

---

## System Overview

```
Phone Camera (MJPEG)
        │
        ▼
  Person Detection          YOLOv8 — persons only
        │
        ▼
  Identity Lock             Appearance Re-ID — stays locked to you, not strangers
        │
        ▼
  Steering Decision         5-zone compass-bearing tracker
        │
        ▼
  Motion Planning           Phase 1 → 2 → 3 lost-person recovery
        │
        ▼
  BLE Command               "MOVE F 2 270" → Elegoo UNO R4 WiFi
        │
        ▼
  Robot Motors              L298N H-bridge → wheels
```

Everything runs on-device. No internet connection. ~10 FPS on CPU.

---

## Hardware

| Part | Purpose |
|---|---|
| Elegoo UNO R4 WiFi | Robot controller (custom BLE firmware) |
| Android phone | Camera + IMU (mounted on robot) |
| Windows laptop | Vision + logic (Python) |

---

## Key Ideas

### 1. Compass-bearing person tracking

Rather than tracking where in the frame the person is (which resets every time the robot turns), the system maintains an **absolute compass bearing** to the person:

```python
# Where is the person in the world, not just in the frame
person_bearing = robot_heading + (cx - 0.5) * HFOV_DEG
```

When the person disappears, the robot knows the exact compass direction they were last seen in — so the search rotation is precise, not a guess.

### 2. Five-zone steering with hysteresis

The frame is divided into five horizontal zones. Zone transitions require the person to hold position across multiple frames before the robot reacts — this prevents jitter from a person swaying slightly.

```
  ──────────────────────────────────────────────────
  HARD_L │ SOFT_L │         CENTER        │ SOFT_R │ HARD_R
         │        │                       │        │
   Fast  │  Slow  │     Move forward      │  Slow  │  Fast
   turn  │  turn  │                       │  turn  │  turn
  ──────────────────────────────────────────────────
```

### 3. Three-phase lost-person recovery

When the person disappears, the system classifies the loss before reacting:

- **Occluded** — briefly hidden (furniture, another person). Robot holds still and sweeps locally.
- **Likely exit** — person was moving toward an edge and shrinking. Robot pursues in that direction.
- **Left room** — person was at the edge, moving away, door visible. Robot enters the new space and searches.

```
Loss detected
    │
    ▼
Classify: occluded / likely_exit / left_room
    │
    ├─ occluded   → Phase 1: local ±sweep toward last-seen side
    │
    ├─ likely_exit → Phase 2: face last known compass bearing, drive forward
    │
    └─ left_room  → Phase 2 + Phase 3: pursue then sweep new room
```

### 4. Appearance Re-Identification

Locking is by appearance, not just position. The system builds a visual profile of the target person split into three body regions (head, torso, legs) and scores each detected person against that profile every frame. This means the robot re-acquires the correct person after they step out from behind an obstacle, even if another person is also visible.

### 5. Metric scale self-calibration

The onboard visual odometry (ORB features + essential matrix) outputs unitless x/y coordinates. The system continuously calibrates a real-world scale factor by comparing how much the robot's VO position changes versus how much the person's estimated distance changes:

```
scale = Δ real_distance / Δ VO_translation
```

After a few seconds of normal following, world coordinates become metric — letting the pursuit phase navigate to a real position rather than a guess.

---

## Tech Stack

| Component | Technology |
|---|---|
| Object detection | YOLOv8 nano (Ultralytics) |
| Re-Identification | OSNet x1.0 (torchreid) |
| Visual odometry | Lucas-Kanade optical flow + ORB monocular VO |
| IMU fusion | Android rotation vector (phone sensors) |
| Scene understanding | VLM-based door/exit classification |
| Robot comms | BLE GATT (bleak + ArduinoBLE) |
| Server | Flask + Waitress |
| Language | Python 3.11 |

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────┐
│                      Laptop (Python)                    │
│                                                         │
│  ┌──────────┐    ┌──────────┐    ┌───────────────────┐  │
│  │  Camera  │───▶│  YOLO    │───▶│  Person Lock      │  │
│  │  Feed    │    │  Detect  │    │  (IoU + ReID)     │  │
│  └──────────┘    └──────────┘    └────────┬──────────┘  │
│                                           │             │
│  ┌──────────┐    ┌──────────┐    ┌────────▼──────────┐  │
│  │  IMU     │───▶│  VO/SLAM │───▶│  Steering Logic   │  │
│  │  (phone) │    │  Fusion  │    │  + Loss Recovery  │  │
│  └──────────┘    └──────────┘    └────────┬──────────┘  │
│                                           │             │
│                                  ┌────────▼──────────┐  │
│                                  │  BLE Command Gate │  │
│                                  │  (debounce + STOP)│  │
│                                  └────────┬──────────┘  │
└───────────────────────────────────────────┼─────────────┘
                                            │ Bluetooth
                                   ┌────────▼──────────┐
                                   │  Elegoo UNO R4    │
                                   │  WiFi (firmware)  │
                                   └────────┬──────────┘
                                            │
                                   ┌────────▼──────────┐
                                   │   Robot Motors    │
                                   └───────────────────┘
```

---

## BLE Command Protocol

Commands are UTF-8 strings sent to the robot over Bluetooth Low Energy:

```
MOVE F 2 270    # Move forward, speed tier 2, for 270ms
TURN L 1 80     # Turn left,  speed tier 1, for 80ms
STOP            # Immediate stop
PING            # Keepalive (robot replies PONG)
```

Speed tiers scale motor PWM (1 = slow, 3 = fast). Duration in milliseconds controls how long each pulse runs before the next command arrives.

---

## Setup (high level)

1. Flash custom firmware to Elegoo UNO R4 WiFi
2. Install IP Webcam on Android phone, mount on robot
3. `pip install flask ultralytics bleak opencv-python torch torchreid`
4. `python server.py`
5. Open `http://localhost:8022` in browser
6. Connect camera → connect BLE → press `1` to lock onto yourself

---

## Project Structure

```
final_pipe/
├── server.py              # Flask server + browser UI
├── ble_instruction.py     # Steering logic + loss recovery
├── reid.py                # Appearance re-identification
├── slam_vo.py             # Visual odometry + SLAM
├── robot.py               # BLE driver
├── calibration.py         # Distance estimation
└── Elegoo_BLE_Control/
    └── *.ino              # Custom Arduino firmware
```

---

## Status

Actively developed. Running on real hardware. Core follow-me loop is stable; VO/SLAM-guided pursuit and multi-room tracking are in testing.
