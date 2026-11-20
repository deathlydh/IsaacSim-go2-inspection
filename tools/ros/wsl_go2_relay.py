"""Relay standard ROS CDR between Windows DDS and WSL DDS over WSL interop pipes.

Run under sourced Ubuntu ROS. The Windows worker is a separate ROS participant
using the Jazzy libraries shipped with Isaac Sim. No inbound ports are needed.
"""

import argparse
import signal
import os
from pathlib import Path
from queue import Empty, Queue
import struct
import subprocess
import sys
import threading


HEADER = struct.Struct("!BI")


def send(pipe, index, payload):
    pipe.write(HEADER.pack(index, len(payload)) + payload)
    pipe.flush()


def read_pipe(pipe, incoming, allowed):
    try:
        while header := pipe.read(HEADER.size):
            index, size = HEADER.unpack(header)
            if index not in allowed or not 0 < size <= 1024 * 1024:
                raise ValueError("Unexpected ROS relay frame")
            payload = pipe.read(size)
            if len(payload) != size:
                raise EOFError("Incomplete ROS relay frame")
            incoming.put((index, payload))
    except (OSError, ValueError, EOFError, struct.error) as exc:
        print(f"Relay pipe stopped: {exc}", file=sys.stderr)
    finally:
        incoming.put(None)


def relay(windows, source, sink, domain):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    streams = [
        (LaserScan, "/scan", qos_profile_sensor_data),
        (Odometry, "/odom", 10),
        (TFMessage, "/tf", 100),
        (TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        (Clock, "/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        (TwistStamped, "/cmd_vel", 1),
    ]
    outbound = (0, 1, 2, 3, 4) if windows else (5,)
    inbound = (5,) if windows else (0, 1, 2, 3, 4)
    incoming = Queue(maxsize=128)
    reader = threading.Thread(target=read_pipe, args=(source, incoming, inbound), daemon=True)
    rclpy.init(args=[], domain_id=domain)
    try:
        node = rclpy.create_node("go2_relay_windows" if windows else "go2_relay_wsl", enable_rosout=False)
        try:
            for index in outbound:
                msg, topic, qos = streams[index]
                node.create_subscription(msg, topic, lambda data, i=index: send(sink, i, data), qos, raw=True)
            publishers = {index: node.create_publisher(*streams[index]) for index in inbound}
            reader.start()
            print(f"{node.get_name()} ready (domain {domain})", file=sys.stderr, flush=True)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                for _ in range(64):
                    try:
                        packet = incoming.get_nowait()
                    except Empty:
                        break
                    if packet is None:
                        return
                    index, payload = packet
                    publishers[index].publish(payload)
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()


def win_worker(args):
    # Use the host interpreter's NumPy wheel. Kit's bundled Python overlay is
    # built against numerical DLLs in the Kit process.
    import numpy

    prefix = Path(args.isaac_dir) / "exts" / "isaacsim.ros2.core" / args.windows_distro
    with os.add_dll_directory(str(prefix / "lib")):
        sys.path.insert(0, str(prefix / "rclpy"))
        os.environ["PATH"] = str(prefix / "lib") + os.pathsep + os.environ.get("PATH", "")
        os.environ["AMENT_PREFIX_PATH"] = str(prefix)
        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        relay(True, sys.stdin.buffer, sys.stdout.buffer, args.kit_domain)


def wsl_worker(args):
    if args.wsl_domain == args.kit_domain:
        raise ValueError("Use different DDS domains to avoid forwarding a topic back to itself")
    worker = subprocess.check_output(["wslpath", "-w", str(Path(__file__).resolve())], text=True).strip()
    command = [args.windows_python, "-u", worker, "--windows-worker", "--isaac-dir", args.isaac_dir,
               "--windows-distro", args.windows_distro, "--kit-domain", str(args.kit_domain)]
    with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE) as process:
        try:
            relay(False, process.stdout, process.stdin, args.wsl_domain)
        finally:
            process.stdin.close()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.terminate()
        if process.returncode:
            raise SystemExit(process.returncode)


if __name__ == "__main__":
    def stop_relay(_signum, _frame):
        raise KeyboardInterrupt

    if not sys.platform.startswith("win"):
        signal.signal(signal.SIGTERM, stop_relay)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--windows-worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--windows-python", default="python.exe")
    parser.add_argument("--isaac-dir", default="D:/isaacsim")
    parser.add_argument("--windows-distro", default="jazzy", choices=("jazzy", "humble"))
    parser.add_argument("--kit-domain", default=0, type=int)
    parser.add_argument("--wsl-domain", default=42, type=int)
    options = parser.parse_args()
    try:
        win_worker(options) if options.windows_worker else wsl_worker(options)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
