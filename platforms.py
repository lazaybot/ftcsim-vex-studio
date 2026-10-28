"""Competition-platform profiles for FTC and VEX-style robots.

The simulator remains a generic MuJoCo model.  A profile supplies the motor
curves, encoder scale, controller vocabulary and default speed limits that sit
above it.  VEX values are representative V5 11 W cartridge figures; teams
should replace them with measured values for their gearing and battery.
"""
from dataclasses import dataclass

from motors import Motor, YJ_60, YJ_312


@dataclass(frozen=True)
class PlatformProfile:
    key: str
    label: str
    drive_motor: Motor
    arm_motor: Motor
    max_drive: float = 1.0
    max_turn: float = 1.0
    controller_name: str = "gamepad"


FTC = PlatformProfile(
    key="ftc",
    label="FIRST Tech Challenge",
    drive_motor=YJ_312,
    arm_motor=YJ_60,
    controller_name="FTC-style gamepad",
)

VEX = PlatformProfile(
    key="vex",
    label="VEX V5",
    drive_motor=Motor("VEX V5 11W 200rpm", free_rpm=200, stall_nm=2.1,
                      ticks_per_rev=1800),
    arm_motor=Motor("VEX V5 11W 100rpm", free_rpm=100, stall_nm=2.1,
                    ticks_per_rev=1800, external_ratio=2.0),
    max_drive=0.90,
    max_turn=0.80,
    controller_name="VEX-style controller",
)

PROFILES = {p.key: p for p in (FTC, VEX)}


def get_profile(name: str) -> PlatformProfile:
    try:
        return PROFILES[name.lower()]
    except KeyError as exc:
        raise ValueError(f"unknown robot mode {name!r}; choose: {', '.join(PROFILES)}") from exc
