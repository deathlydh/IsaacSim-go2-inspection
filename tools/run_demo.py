"""Windows entry point for the existing Isaac + WSL Nav2 demo."""
import argparse
from datetime import datetime, timezone
from html import escape
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]


def run_mapping(command, output):
    output.mkdir(parents=True, exist_ok=False)
    with subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL) as process:
        try:
            while not (output / "mapping.ready").exists():
                if process.poll() is not None:
                    return process.returncode
                time.sleep(0.2)
            input("Drive in Isaac, then park and press Enter here to save the map. ")
            (output / "save.request").touch(exist_ok=False)
            return process.wait()
        except (KeyboardInterrupt, EOFError):
            (output / "cancel.request").touch()
            print("Mapping canceled.", file=sys.stderr)
            process.wait()
            return 130


def run(mode, *, rviz=False):
    isaac = Path(os.environ.get("GO2_ISAAC_DIR", "D:/isaacsim")).resolve()
    client = isaac / "skills/isaac-sim-remote/scripts/isaacsim_send.py"
    if not client.is_file():
        raise RuntimeError(f"Isaac remote client missing: {client}. Check GO2_ISAAC_DIR.")
    import numpy  # noqa: F401 - required by the native ROS relay
    if mode in ("nav", "obstacle"):
        import cv2  # noqa: F401 - used by the report's CV subprocess
    distro = os.environ.get("GO2_WSL_DISTRO", "Ubuntu-22.04")
    wsl = ["wsl.exe", "-d", distro, "--"]
    root_wsl = subprocess.check_output(
        [*wsl, "wslpath", "-u", ROOT.as_posix()], stdin=subprocess.DEVNULL, text=True, timeout=30
    ).strip()
    python_wsl = subprocess.check_output(
        [*wsl, "wslpath", "-u", Path(sys.executable).as_posix()], stdin=subprocess.DEVNULL,
        text=True, timeout=30
    ).strip()
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
    output = ROOT / "reports" / "demo" / run_id
    print(f"Results: {output}", flush=True)
    command = [*wsl, "bash", f"{root_wsl}/tools/ros/demo_episode.sh", mode,
               f"{root_wsl}/reports/demo/{run_id}", python_wsl, isaac.as_posix()]
    if rviz:
        command.append("--rviz")
    if mode == "mapping":
        return run_mapping(command, output)
    status = subprocess.run(command, cwd=ROOT).returncode
    if mode == "rviz":
        return status
    result_path = output / "nav2/result.json"
    if result_path.is_file():
        result = json.loads(result_path.read_text(encoding="utf-8"))
        frame = output / "nav2/front.png"
        message = "CV not run."
        overlay = ""
        if status == 0 and result.get("action_status") == "SUCCEEDED" and frame.is_file():
            check = subprocess.run(
                [sys.executable, "-m", "cv.read_image", str(frame), "--output-root", str(output / "cv")],
                cwd=ROOT,
            )
            status = check.returncode
            readings = sorted((output / "cv").glob("*/result.json"))
            if check.returncode == 0 and len(readings) == 1:
                reading = json.loads(readings[0].read_text(encoding="utf-8"))
                message = (f'{reading["value"]:.3g} {reading["unit"]}'
                           if reading["read_status"] == "ok" else str(reading["error"]))
                image_path = readings[0].with_name("annotated.png").relative_to(output).as_posix()
                overlay = f'<p><img width="720" src="{escape(image_path, quote=True)}" alt="CV annotation"></p>'
            else:
                message = "CV failed. Check the terminal output."
                status = 1
        (output / "index.html").write_text(
            '<!doctype html><meta charset="utf-8"><title>Go2 Inspection</title>'
            '<h1>Go2 Inspection</h1>'
            f'<p>Navigation: {escape(str(result.get("action_status") or result.get("error")))}</p>'
            f'<p>Experimental CV for Gauge_01: {escape(message)}</p>'
            '<p><a href="nav2/result.json">Navigation log</a></p>'
            + ('<p><a href="nav2/front.png">Original frame</a></p>' if frame.is_file() else '')
            + overlay, encoding="utf-8",
        )
        print(f"Open: {output / 'index.html'}")
    elif status == 0:
        print(f"Navigation did not write a result: {result_path}", file=sys.stderr)
        return 1
    return status


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["nav", "cancel", "obstacle", "mapping", "rviz"])
    parser.add_argument("--rviz", action="store_true", help="open a viewer for the Nav2 run")
    args = parser.parse_args()
    if args.rviz and args.mode not in ("nav", "cancel", "obstacle"):
        parser.error("--rviz is only available with nav, cancel or obstacle")
    try:
        raise SystemExit(run(args.mode, rviz=args.rviz))
    except (OSError, RuntimeError, ImportError, subprocess.SubprocessError) as error:
        print(f"Demo setup failed: {error}", file=sys.stderr)
        raise SystemExit(1)
    except KeyboardInterrupt:
        print("Interrupted. Press Esc in Isaac to return to manual if needed.", file=sys.stderr)
        raise SystemExit(130)
