"""Drive the robot yourself. One window, no surprises.

    python teleop.py

    W / S      forward / back          SPACE   toggle claw
    Z / X      claw tilt down / up     1       snap to grab pose
    A / D      strafe left / right     G       field-centric on/off
    Q / E      turn left / right       BKSP    reset robot
    H          gyro heading-hold
    R / F      arm up / down           ESC     quit

    left drag    orbit camera          scroll  zoom
    right drag   pan camera             HOME    recentre on robot
    TAB          cycle camera preset

Plug in an Xbox-style controller and it works alongside the keyboard: left
stick drives, right stick X turns, triggers work the arm, A/B the claw.

WHY THERE IS NO MUJOCO VIEWER WINDOW
    MuJoCo's built-in viewer binds every letter A-Z to a render flag -- W is
    wireframe, S is shadow, R is reflection, G is fog. Those are handled inside
    its C library, so they fire whenever that window has focus and Python cannot
    intercept or undo them. Driving with WASD anywhere near it turns the robot
    into a wireframe grid.

    So we don't use it. MuJoCo renders offscreen, we blit the image into our own
    pygame window, and we own every key and mouse event. The camera controls
    below are ours too. Costs a few frames per second; worth it.
"""
import argparse
import os
import sys

os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")

import time

import mujoco
import numpy as np
import pygame

from robot import build_mjcf
from motors import Robot
from version import describe
import robotconfig as rc
from autonomous import Path, PathFollower, wrap
from platforms import get_profile
from telemetry import Telemetry
import theme

WIN_W, WIN_H = 1180, 740
DEADZONE = 0.08
FRAME = 1.0 / 60.0

# name, distance, elevation, azimuth-follows-robot
CAMERAS = [
    ("chase",   1.55, -18, True),
    ("close",   0.95, -14, True),
    ("overhead", 2.60, -78, False),
    ("wide",    3.20, -28, False),
]


def deadzone(v):
    return 0.0 if abs(v) < DEADZONE else (abs(v) - DEADZONE) / (1 - DEADZONE) * np.sign(v)


class Sim:
    def __init__(self, config=None, mode="ftc", auto_path=None):
        pygame.init()
        self.profile = get_profile(mode)
        pygame.display.set_caption(f"Robot Simulation Studio — {self.profile.label}")
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.font = pygame.font.SysFont("monospace", 15)
        self.big = pygame.font.SysFont("monospace", 19, bold=True)

        pygame.joystick.init()
        self.js = None
        if pygame.joystick.get_count():
            self.js = pygame.joystick.Joystick(0)
            self.js.init()

        self.config = config or rc.default_config()
        self.model = mujoco.MjModel.from_xml_string(
            build_mjcf(wheel_preset=self.config.wheel_preset, quality="fast"))
        self.data = mujoco.MjData(self.model)
        self.bot = Robot(self.model, self.data, config=self.config, profile=self.profile)
        self.bot.stow()

        self.renderer = mujoco.Renderer(self.model, height=WIN_H, width=WIN_W)
        self.cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.cam)
        self.cam_idx = 0
        self.az_offset = 0.0
        self.pan = np.zeros(3)      # camera target offset from the robot
        self.el = CAMERAS[0][2]
        self.dist = CAMERAS[0][1]

        self.grip = 0.0
        self.grip_closed = False
        self.field_centric = False
        self.heading_hold = True
        self.arm_target = 1.2
        self.manual_arm = False
        # How far the claw is tilted below level. The wrist servo tracks the
        # shoulder, so 0 keeps the claw horizontal at any arm angle. Level is
        # right for driving, but you CANNOT pick anything off the floor with it:
        # the wrist block grounds out on the deck with the claw tip at 0.083 m,
        # and the game element's top is at 0.076 m. Tilting the claw down lets
        # the arm run to its joint limit with the tip at floor level instead.
        self.wrist_offset = 0.0
        self.running = True
        self.fps = 0.0
        # Resolved once: git_suffix() shells out, and we draw every frame.
        self.version = describe()
        self.telemetry = Telemetry()
        self.show_graphs = True
        self.auto = PathFollower(Path.load(auto_path)) if auto_path else None

    # -- input ----------------------------------------------------------

    def handle_events(self):
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                self.running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    self.running = False
                elif e.key == pygame.K_SPACE:
                    self.grip_closed = not self.grip_closed
                elif e.key == pygame.K_g:
                    self.field_centric = not self.field_centric
                elif e.key == pygame.K_h:
                    self.heading_hold = not self.heading_hold
                    self.bot.heading_target = None
                elif e.key == pygame.K_t:
                    self.show_graphs = not self.show_graphs
                elif e.key == pygame.K_c:
                    self.telemetry.export_csv("telemetry.csv")
                elif e.key == pygame.K_p and self.auto:
                    self.auto.index = 0
                elif e.key == pygame.K_BACKSPACE:
                    mujoco.mj_resetData(self.model, self.data)
                    self.bot.stow()
                    self.arm_target, self.grip = 1.2, 0.0
                elif e.key == pygame.K_1:
                    # Grab preset: arm down, claw pointed at the floor, open.
                    # Measured working values, same as auto_demo uses.
                    self.arm_target = -0.44
                    self.wrist_offset = 1.45
                    self.manual_arm = False
                    self.grip_closed = False
                elif e.key == pygame.K_2:
                    self.arm_target = 1.2          # travel / stow pose
                    self.wrist_offset = 0.0
                    self.manual_arm = False
                elif e.key == pygame.K_TAB:
                    self.cam_idx = (self.cam_idx + 1) % len(CAMERAS)
                    _, self.dist, self.el, _ = CAMERAS[self.cam_idx]
                    self.az_offset = 0.0
                    self.pan[:] = 0.0
                elif e.key == pygame.K_HOME:
                    self.pan[:] = 0.0
                    self.az_offset = 0.0
            elif e.type == pygame.MOUSEWHEEL:
                self.dist = float(np.clip(self.dist * (0.9 ** e.y), 0.4, 6.0))
            elif e.type == pygame.MOUSEMOTION and e.buttons[0]:
                self.az_offset += e.rel[0] * 0.4
                self.el = float(np.clip(self.el - e.rel[1] * 0.3, -89, 5))
            elif e.type == pygame.MOUSEMOTION and (e.buttons[2] or e.buttons[1]):
                self.pan_camera(e.rel[0], e.rel[1])
            elif e.type == pygame.JOYBUTTONDOWN and self.js:
                if e.button == 0:
                    self.grip_closed = True
                elif e.button == 1:
                    self.grip_closed = False
                elif e.button == 2:
                    self.field_centric = not self.field_centric

    def sticks(self):
        k = pygame.key.get_pressed()
        axis = lambda a, b: float(k[a]) - float(k[b])
        out = dict(
            fwd=axis(pygame.K_w, pygame.K_s),
            strafe=axis(pygame.K_d, pygame.K_a),
            turn=axis(pygame.K_e, pygame.K_q),
            arm=axis(pygame.K_r, pygame.K_f),
            wrist=axis(pygame.K_z, pygame.K_x),
        )
        # Gamepad ADDS to the keyboard rather than replacing it, so an idle or
        # phantom controller can't silently swallow WASD.
        if self.js:
            ax = lambda i: deadzone(self.js.get_axis(i)) if self.js.get_numaxes() > i else 0.0
            lt = (self.js.get_axis(4) + 1) / 2 if self.js.get_numaxes() > 4 else 0.0
            rt = (self.js.get_axis(5) + 1) / 2 if self.js.get_numaxes() > 5 else 0.0
            pad = dict(fwd=-ax(1), strafe=ax(0), turn=ax(3), arm=rt - lt)
            pad["wrist"] = 0.0
            for key, v in pad.items():
                if abs(v) > abs(out[key]):
                    out[key] = v
        return out

    def pan_camera(self, dx, dy):
        """Slide the camera sideways/up-down in its own screen plane.

        The target is stored as an OFFSET from the robot, not an absolute
        point, because draw() re-aims the camera at the chassis every frame --
        an absolute target would be overwritten instantly. Storing an offset
        means panning survives while the robot keeps being followed.

        Scaled by distance so a drag moves the view the same amount on screen
        whether you're zoomed right in or way out.
        """
        az = np.radians(self.cam.azimuth)
        el = np.radians(self.cam.elevation)

        # Screen-plane basis for the current camera orientation.
        right = np.array([-np.sin(az), np.cos(az), 0.0])
        up = np.array([-np.sin(el) * np.cos(az),
                       -np.sin(el) * np.sin(az),
                       np.cos(el)])

        scale = self.dist * 0.0016
        # Drag the world with the cursor: moving right pushes the target left.
        self.pan -= right * dx * scale
        self.pan += up * dy * scale

    # -- physics --------------------------------------------------------

    def step_physics(self, s, n):
        for _ in range(n):
            if self.auto and not self.auto.finished:
                s = {**s, **self.auto.command(self.bot.pose())}
            s = {**s, "turn": s["turn"] * self.profile.max_turn}
            fwd, strafe = s["fwd"], s["strafe"]
            if self.field_centric:
                h = self.bot.heading()
                fwd, strafe = (fwd * np.cos(-h) - strafe * np.sin(-h),
                               fwd * np.sin(-h) + strafe * np.cos(-h))
            if self.heading_hold:
                self.bot.drive_held(fwd, strafe, s["turn"])
            else:
                self.bot.drive(fwd, strafe, s["turn"])

            self.bot.drive_power *= self.profile.max_drive

            if abs(s["arm"]) > 0.05:
                self.manual_arm = True
                self.bot.set_arm(s["arm"])
            else:
                if self.manual_arm:
                    self.arm_target = float(self.data.joint("shoulder").qpos[0])
                    self.manual_arm = False
                self.bot.hold_arm(self.arm_target)
            if abs(s.get("wrist", 0.0)) > 0.05:
                self.wrist_offset = float(np.clip(
                    self.wrist_offset + s["wrist"] * 0.012, 0.0, 1.55))
            self.bot.level_wrist(self.wrist_offset)

            self.grip = (min(1.0, self.grip + 0.02) if self.grip_closed
                         else max(0.0, self.grip - 0.02))
            self.bot.set_grip(self.grip)

            self.bot.apply()
            mujoco.mj_step(self.model, self.data)

    # -- drawing --------------------------------------------------------

    def draw(self):
        name, _, _, follows = CAMERAS[self.cam_idx]
        _, _, heading = self.bot.pose()
        base_az = np.degrees(heading) + 180 if follows else 90
        self.cam.azimuth = base_az + self.az_offset
        self.cam.elevation = self.el
        self.cam.distance = self.dist
        self.cam.lookat[:] = self.data.body("chassis").xpos + self.pan

        self.renderer.update_scene(self.data, self.cam)
        # frombuffer avoids a full array transpose; make_surface is ~9x slower.
        frame = pygame.image.frombuffer(
            self.renderer.render().tobytes(), (WIN_W, WIN_H), "RGB")
        self.screen.blit(frame, (0, 0))
        vx, vy = self.data.body("chassis").cvel[3:5]
        heading_error = 0.0 if self.bot.heading_target is None else wrap(self.bot.heading_target - heading)
        self.telemetry.add(speed=float(np.hypot(vx, vy)), heading_error=float(heading_error),
                           arm=float(self.data.joint("shoulder").qpos[0]))
        if self.show_graphs:
            self.telemetry.draw(pygame, self.screen, pygame.Rect(WIN_W - 340, 64, 326, 152))
        self.hud(name)
        pygame.display.flip()

    def hud(self, cam_name):
        x, y, h = self.bot.pose()
        arm = float(self.data.joint("shoulder").qpos[0])
        # Product-style header establishes mode and state without covering the scene.
        header = pygame.Rect(14, 12, WIN_W - 28, 42)
        theme.panel(pygame, self.screen, header)
        self.screen.blit(self.big.render("ROBOT SIMULATION STUDIO", True, theme.TEXT), (28, 23))
        badge_text = f"{self.profile.key.upper()}  •  {self.config.drivetrain.upper()}"
        badge = self.font.render(badge_text, True, theme.CYAN)
        self.screen.blit(badge, (WIN_W - badge.get_width() - 30, 25))

        info = pygame.Rect(14, 64, 270, 286)
        theme.panel(pygame, self.screen, info)

        mode = "FIELD-CENTRIC" if self.field_centric else "ROBOT-CENTRIC"
        rows = [
            (self.big, "ROBOT STATE", theme.MUTED),
            (self.font, mode, theme.CYAN if self.field_centric else theme.TEXT),
            (self.font, f"POSITION    {x:+.2f}, {y:+.2f} m", theme.TEXT),
            (self.font, f"HEADING     {np.degrees(h):+7.1f} deg", theme.TEXT),
            (self.font, f"ARM         {arm:+.2f} rad", theme.TEXT),
            (self.font, f"WRIST       {self.wrist_offset:+.2f} rad", theme.TEXT),
            (self.font, f"claw     {'CLOSED' if self.grip > 0.5 else 'open'}",
             theme.ORANGE if self.grip > 0.5 else theme.TEXT),
            (self.font, f"HEADING HOLD {'ON' if self.heading_hold else 'OFF'}",
             theme.GREEN if self.heading_hold else theme.MUTED),
            (self.font, f"CAMERA       {cam_name}"
                        f"{'  (panned)' if np.linalg.norm(self.pan) > 0.01 else ''}",
             theme.ORANGE if np.linalg.norm(self.pan) > 0.01 else theme.MUTED),
            (self.font, f"PERFORMANCE  {f'{self.fps:.0f} FPS' if self.fps else 'WARMING UP'}"
                        f"{'  • CONTROLLER' if self.js else ''}", theme.MUTED),
        ]
        yy = 78
        for font, text, colour in rows:
            self.screen.blit(font.render(text.upper(), True, colour), (28, yy))
            yy += 24

        # Version box first, bottom-right. Dimmed so it never competes with the
        # telemetry, but always present so anyone can say which build they're
        # looking at.
        vs = self.font.render(self.version, True, (130, 136, 148))
        vbox_w = vs.get_width() + 22
        vbox_x = WIN_W - vbox_w - 14
        vbg = pygame.Surface((vbox_w, 30), pygame.SRCALPHA); vbg.fill(theme.PANEL)
        self.screen.blit(vbg, (vbox_x, WIN_H - 44))
        self.screen.blit(vs, (vbox_x + 11, WIN_H - 37))

        # Controls hint, trimmed so it can never run under the version box.
        # Dropping hints from the end beats overlapping text, which is what
        # happened when both were drawn at fixed positions.
        parts = ["WASD drive", "QE turn", "RF arm", "ZX claw tilt",
                 "SPACE claw", "1 grab pose", "2 stow", "G field", "H hold",
                 "|", "L-drag orbit", "R-drag pan", "TAB cam"]
        budget = vbox_x - 14 - 30          # leave a gap before the version box
        while parts:
            surf = self.font.render("  ".join(parts), True, (150, 156, 166))
            if surf.get_width() + 24 <= budget:
                break
            parts.pop()
        if parts:
            bg = pygame.Surface((surf.get_width() + 24, 30), pygame.SRCALPHA)
            bg.fill(theme.PANEL)
            self.screen.blit(bg, (14, WIN_H - 44))
            self.screen.blit(surf, (26, WIN_H - 37))

    # -- main loop ------------------------------------------------------

    def run(self, max_frames=None, screenshot=None):
        dt = self.model.opt.timestep
        acc = 0.0
        last = time.perf_counter()
        smooth = 0.0

        frames = 0
        while self.running:
            now = time.perf_counter()
            elapsed = now - last
            last = now
            acc += min(elapsed, 0.25)          # cap so a stall can't spiral
            smooth = 0.9 * smooth + 0.1 * elapsed
            self.fps = 1.0 / smooth if smooth > 0 and frames > 10 else 0.0

            self.handle_events()
            s = self.sticks()

            # Run however many fixed physics steps fit the real time elapsed, so
            # holding a key produces continuous motion at true 1x speed.
            steps = 0
            while acc >= dt and steps < 200:
                self.step_physics(s, 1)
                acc -= dt
                steps += 1

            self.draw()
            frames += 1
            if screenshot and frames == 1:
                pygame.image.save(self.screen, screenshot)
            if max_frames is not None and frames >= max_frames:
                self.running = False

            slack = FRAME - (time.perf_counter() - now)
            if slack > 0:
                time.sleep(slack)

        pygame.quit()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", help="robot_config.json from configure.py")
    ap.add_argument("--mode", choices=("ftc", "vex"), default="ftc",
                    help="hardware profile; VEX changes motor curves and speed limits")
    ap.add_argument("--path", help="autonomous path JSON from path_editor.py")
    ap.add_argument("--smoke-test", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()

    cfg = rc.RobotConfig.load(args.config) if args.config else None
    print(f"ftcsim {describe()} - teleop")
    if cfg:
        print(f"config: {args.config}\n")
        print(cfg.summary())
    print("\nWASD drive  QE turn  RF arm  ZX claw tilt  SPACE claw  1 grab  2 stow")
    Sim(config=cfg, mode=args.mode, auto_path=args.path).run(
        max_frames=3 if args.smoke_test else None, screenshot=args.screenshot)
