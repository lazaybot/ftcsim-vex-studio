"""JSON waypoint format and a conservative holonomic path follower."""
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path as FilePath


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Waypoint:
    x: float
    y: float
    heading: float = 0.0
    speed: float = 0.65
    tolerance: float = 0.08


class Path:
    def __init__(self, waypoints=None, name="untitled"):
        self.name = name
        self.waypoints = list(waypoints or [])

    def save(self, filename):
        FilePath(filename).write_text(json.dumps({
            "format": "ftcsim-path-v1", "name": self.name,
            "waypoints": [asdict(w) for w in self.waypoints],
        }, indent=2) + "\n")

    @classmethod
    def load(cls, filename):
        raw = json.loads(FilePath(filename).read_text())
        if raw.get("format") != "ftcsim-path-v1":
            raise ValueError("unsupported autonomous path format")
        return cls([Waypoint(**w) for w in raw["waypoints"]], raw.get("name", "untitled"))


class PathFollower:
    def __init__(self, path, position_kp=1.5, heading_kp=2.2):
        self.path = path
        self.position_kp = position_kp
        self.heading_kp = heading_kp
        self.index = 0

    @property
    def finished(self):
        return self.index >= len(self.path.waypoints)

    def command(self, pose):
        if self.finished:
            return {"fwd": 0.0, "strafe": 0.0, "turn": 0.0}
        x, y, heading = pose
        target = self.path.waypoints[self.index]
        dx, dy = target.x - x, target.y - y
        distance = math.hypot(dx, dy)
        if distance <= target.tolerance:
            self.index += 1
            return self.command(pose)
        # World vector -> robot frame.  This output plugs directly into mix().
        fwd = math.cos(heading) * dx + math.sin(heading) * dy
        strafe = -math.sin(heading) * dx + math.cos(heading) * dy
        scale = min(target.speed, self.position_kp * distance) / max(distance, 1e-9)
        return {
            "fwd": max(-1.0, min(1.0, fwd * scale)),
            "strafe": max(-1.0, min(1.0, strafe * scale)),
            "turn": max(-1.0, min(1.0, self.heading_kp * wrap(target.heading - heading))),
        }
