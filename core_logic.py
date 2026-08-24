"""
core_logic.py — Simplified illustration of the follow-me steering logic.

This is a stripped-down version showing the key ideas:
  1. Zone-based steering from camera position
  2. Compass bearing to person (heading-aware)
  3. Lost-person recovery phases

The real pipeline has hysteresis, IMU fusion, ReID, SLAM, and more —
this is the conceptual skeleton.
"""

import math
import time


# ── Constants ─────────────────────────────────────────────────────────────────

HFOV_DEG    = 65.0    # camera horizontal field of view
SOFT_TURN   = 0.35    # cx < this → turn left  /  cx > (1 - this) → turn right
HARD_TURN   = 0.20    # cx < this → hard turn left
STOP_RATIO  = 0.40    # bbox area / frame area — person too close, stop

MISS_GRACE  = 3       # frames to hold still before searching
GIVE_UP     = 90      # frames before giving up entirely


# ── 1. Zone steering ──────────────────────────────────────────────────────────

def get_zone(cx: float) -> str:
    """
    Map horizontal person position (0=left edge, 1=right edge) to a zone.
    cx = 0.5 means perfectly centred.
    """
    if cx < HARD_TURN:
        return "hard_left"
    elif cx < SOFT_TURN:
        return "soft_left"
    elif cx > (1 - HARD_TURN):
        return "hard_right"
    elif cx > (1 - SOFT_TURN):
        return "soft_right"
    else:
        return "center"


def zone_to_command(zone: str, area_ratio: float) -> str:
    """
    Convert zone + proximity into a BLE motor command.
    area_ratio is bbox_area / frame_area — higher means person is closer.
    """
    if area_ratio >= STOP_RATIO:
        return "STOP"                   # too close — hold position

    commands = {
        "hard_left":  "TURN L 2 120",  # fast left turn
        "soft_left":  "TURN L 1 80",   # gentle left
        "center":     "MOVE F 2 270",  # straight ahead
        "soft_right": "TURN R 1 80",
        "hard_right": "TURN R 2 120",
    }
    return commands[zone]


# ── 2. Compass bearing ────────────────────────────────────────────────────────

def person_bearing(robot_heading_deg: float, cx: float) -> float:
    """
    Compute absolute compass direction the person is standing in.

    robot_heading_deg — where the robot is pointing (from IMU/VO)
    cx               — person horizontal position in frame (0–1)

    Result: compass bearing 0–360°, same coordinate system as robot heading.
    This persists across robot rotations — if robot spins, bearing stays correct.
    """
    offset = (cx - 0.5) * HFOV_DEG     # negative = left, positive = right
    bearing = (robot_heading_deg + offset) % 360.0
    return bearing


def bearing_error(target_deg: float, current_deg: float) -> float:
    """Signed angle from current heading to target. Negative = turn left."""
    err = (target_deg - current_deg + 180) % 360 - 180
    return err


# ── 3. Lost-person recovery ───────────────────────────────────────────────────

class FollowState:
    """Minimal state machine for person-following."""

    def __init__(self):
        self.miss_frames      = 0
        self.last_bearing     = 0.0    # compass bearing when person last seen
        self.last_cx          = 0.5
        self.last_area        = 0.2

    def update(self, person: dict | None, robot_heading: float) -> str:
        """
        Call every frame with the detected person dict (or None if lost).
        Returns a BLE command string.
        """

        # ── Person visible ────────────────────────────────────────────────────
        if person is not None:
            cx         = person["cx"]
            area_ratio = person["area_ratio"]

            # Update memory
            self.miss_frames  = 0
            self.last_cx      = cx
            self.last_area    = area_ratio
            self.last_bearing = person_bearing(robot_heading, cx)

            zone = get_zone(cx)
            return zone_to_command(zone, area_ratio)

        # ── Person lost ───────────────────────────────────────────────────────
        self.miss_frames += 1

        # Grace period — maybe just a momentary occlusion
        if self.miss_frames <= MISS_GRACE:
            return "STOP"

        # Gave up
        if self.miss_frames > GIVE_UP:
            return "STOP"

        # Classify the loss
        loss_type = self._classify_loss()

        if loss_type == "occluded":
            # Phase 1: sweep toward the side they were last seen on
            direction = "R" if self.last_cx >= 0.5 else "L"
            return f"TURN {direction} 1 80"

        else:
            # Phase 2: face the last known compass bearing and move toward it
            err = bearing_error(self.last_bearing, robot_heading)
            if abs(err) > 15:
                direction = "R" if err > 0 else "L"
                return f"TURN {direction} 1 100"
            else:
                return "MOVE F 1 300"   # heading correct — advance into new space

    def _classify_loss(self) -> str:
        """
        Simple heuristic: if person was at the edge of frame AND was small
        (far away / shrinking), they probably left the room.
        Otherwise they're probably just behind something.
        """
        at_edge    = self.last_cx < 0.25 or self.last_cx > 0.75
        far_away   = self.last_area < 0.08

        if at_edge and far_away:
            return "left_room"
        elif at_edge:
            return "likely_exit"
        else:
            return "occluded"


# ── Example usage ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    state = FollowState()

    # Simulate: person visible and centred → drifts right → disappears
    frames = [
        {"cx": 0.50, "area_ratio": 0.12},   # centred
        {"cx": 0.55, "area_ratio": 0.11},   # slight right drift
        {"cx": 0.70, "area_ratio": 0.09},   # moving right
        {"cx": 0.85, "area_ratio": 0.07},   # hard right, shrinking
        None,                                # lost
        None,
        None,
        None,
        None,
    ]

    robot_heading = 45.0   # robot facing NE

    print(f"{'Frame':<6} {'Person':<20} {'Command'}")
    print("-" * 45)
    for i, person in enumerate(frames):
        cmd = state.update(person, robot_heading)
        person_str = f"cx={person['cx']:.2f}" if person else "LOST"
        print(f"{i:<6} {person_str:<20} {cmd}")
