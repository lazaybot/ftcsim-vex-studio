import math
import tempfile
import unittest
from pathlib import Path as FilePath

from autonomous import Path, PathFollower, Waypoint, wrap
from platforms import get_profile
from telemetry import Telemetry


class ExtensionTests(unittest.TestCase):
    def test_profiles_are_behaviorally_different(self):
        ftc, vex = get_profile("ftc"), get_profile("vex")
        self.assertNotEqual(ftc.drive_motor.free_rpm, vex.drive_motor.free_rpm)
        self.assertLess(vex.max_drive, ftc.max_drive)

    def test_path_round_trip_and_follower(self):
        route = Path([Waypoint(1.0, 0.0, math.pi / 2)], "test")
        with tempfile.TemporaryDirectory() as folder:
            filename = FilePath(folder) / "route.json"
            route.save(filename)
            loaded = Path.load(filename)
        cmd = PathFollower(loaded).command((0.0, 0.0, 0.0))
        self.assertGreater(cmd["fwd"], 0)
        self.assertAlmostEqual(cmd["strafe"], 0)
        self.assertGreater(cmd["turn"], 0)

    def test_angle_wrap(self):
        self.assertAlmostEqual(wrap(3 * math.pi), -math.pi)

    def test_telemetry_csv(self):
        telemetry = Telemetry(); telemetry.add(speed=1.25, arm=0.2)
        with tempfile.TemporaryDirectory() as folder:
            filename = telemetry.export_csv(FilePath(folder) / "log.csv")
            text = filename.read_text()
        self.assertIn("speed", text)
        self.assertIn("1.25", text)


if __name__ == "__main__":
    unittest.main()
