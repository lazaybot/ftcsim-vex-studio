"""One entry point for the simulator and engineering tools."""
import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description="FTC/VEX MuJoCo robot simulator")
    commands = parser.add_subparsers(dest="command", required=True)
    drive = commands.add_parser("drive", help="launch interactive simulation")
    drive.add_argument("--mode", choices=("ftc", "vex"), default="ftc")
    drive.add_argument("--path")
    editor = commands.add_parser("edit-path", help="launch autonomous editor")
    editor.add_argument("path", nargs="?")
    commands.add_parser("controller", help="launch controller diagnostics")
    commands.add_parser("check", help="run headless physics/render smoke test")
    args = parser.parse_args()

    if args.command == "drive":
        cmd = [sys.executable, "teleop.py", "--mode", args.mode]
        if args.path: cmd += ["--path", args.path]
    elif args.command == "edit-path":
        cmd = [sys.executable, "path_editor.py"] + ([args.path] if args.path else [])
    elif args.command == "controller":
        cmd = [sys.executable, "controller_test.py"]
    else:
        cmd = [sys.executable, "check.py"]
    raise SystemExit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
