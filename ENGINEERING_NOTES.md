# Engineering extension notes

This project is derived from Austin's MIT-licensed `ftcsim` project. The
original copyright and license are preserved in `LICENSE`; new work adds a
platform layer and engineering tools rather than claiming authorship of the
MuJoCo robot, drivetrain model, or original teleoperation interface.

## Architecture discovered

- `robot.py` generates the MuJoCo scene and robot geometry.
- `motors.py` translates normalized commands into torque using a linear DC
  motor torque-speed curve, and exposes encoder/IMU-like robot state.
- `robotconfig.py` maps named drivetrain positions to model joints.
- `teleop.py` owns input, fixed-step physics, rendering and the HUD.

The safest extension seam is therefore above the MuJoCo model: profiles supply
hardware behavior, the autonomous follower supplies the same normalized drive
commands as a human controller, and telemetry observes state without changing
the dynamics.

## Added components

| File | Purpose |
|---|---|
| `platforms.py` | FTC and representative VEX V5 motor/controller profiles |
| `autonomous.py` | Versioned JSON waypoint format and robot-frame follower |
| `path_editor.py` | Click/drag field editor with save and delete controls |
| `telemetry.py` | Bounded sample buffer, live graphs and CSV export |
| `controller_test.py` | Raw/filtered axes, buttons, hats and event log |
| `tests/test_extensions.py` | Profile, path, angle and CSV regression tests |

## Validation boundary

Unit tests validate data flow and controller math. They do not validate VEX
motor constants, real-field odometry, wheel slip or path-following accuracy.
Those require physical measurements. The VEX defaults should be treated as a
starting estimate and replaced with the robot's actual cartridge, external
gearing, wheel, battery and mass data.

## Verified runtime

The extension was run on macOS arm64 using Python 3.12.16, MuJoCo 3.17.0 and
Pygame 2.6.1. Verification included the four extension unit tests, Python
bytecode compilation, the upstream physics/render check, and three-frame GUI
smoke runs in both FTC and VEX/autonomous modes. The physics check loaded 51
bodies, 50 joints, 80 geometries and 8 actuators and rendered two inspection
images successfully.
