#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
mode=$1
output=$2
export GO2_WINDOWS_PYTHON=$3
isaac_dir=$4
export GO2_KIT_SEND="$isaac_dir/skills/isaac-sim-remote/scripts/isaacsim_send.py"
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tools/ros/go2_fastdds_udp.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
mkdir -p "$output"
exec 9>/tmp/go2-demo-42.lock
flock -n 9 || { echo "Another demo is running." >&2; exit 1; }
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'Install ROS 2 Humble in this WSL distribution first. See README.' >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
packages=(nav2_bringup nav2_map_server)
if [[ "$mode" == mapping ]]; then packages+=(slam_toolbox); fi
for package in "${packages[@]}"; do
    if ! ros2 pkg prefix "$package" >/dev/null; then
        echo "Missing ROS package $package. See README." >&2
        exit 1
    fi
done
kit() {
    local script=$1
    shift
    "$GO2_WINDOWS_PYTHON" -I "$GO2_KIT_SEND" \
        --file "$(wslpath -m "$PWD/tools/isaac/$script.py")" \
        --arg "project_root=$(wslpath -m "$PWD")" --timeout 25 "$@"
}
terminate() {
    local pid=$1
    kill -TERM "$pid" 2>/dev/null || true
    for _ in {1..60}; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
    done
    kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}
demo_started=0
obstacle_added=0
relay_pid=
nav_pid=
cleanup() {
    local status=$?
    trap - EXIT
    if [[ $demo_started == 1 ]]; then
        kit nav2_inspection --arg action=manual >>"$output/kit.log" 2>&1 || {
            echo 'Could not confirm manual mode. Press Esc in Isaac.' >&2
            status=1
        }
    fi
    if [[ $obstacle_added == 1 ]]; then
        kit nav2_inspection --arg action=obstacle_remove >>"$output/kit.log" 2>&1 || {
            echo 'Could not remove the test obstacle. See kit.log.' >&2
            status=1
        }
    fi
    if [[ -n $nav_pid ]]; then terminate "$nav_pid"; fi
    if [[ -n $relay_pid ]]; then terminate "$relay_pid"; fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

echo 'Checking the open Isaac factory...'
if ! kit navigation_session --arg action=status >"$output/kit.log" 2>&1; then
    if grep -q 'Navigation is not attached' "$output/kit.log"; then
        kit navigation_session --arg rmw=rmw_fastrtps_cpp >>"$output/kit.log" 2>&1 || {
            cat "$output/kit.log" >&2
            exit 1
        }
    else
        cat "$output/kit.log" >&2
        echo 'Open start_factory.bat and wait for the robot.' >&2
        exit 1
    fi
fi
# The physics subscription needs time to obtain its first scan after attach.
for _ in {1..8}; do
    kit nav2_inspection --arg action=status >"$output/start.json"
    if grep -q '"scan_pose"' "$output/start.json"; then break; fi
    sleep 1
done
python3 - "$output/start.json" "$mode" <<'PY'
import json
import math
import sys
from pathlib import Path

status = json.loads(Path(sys.argv[1]).read_text().splitlines()[-1])
pose = status.get("scan_pose")
spawn = json.loads(Path("scenes/factory_hall_meta.json").read_text())["robot_spawn"]
if (not status["playing"] or status["owner"] != "manual" or any(status["command"])
        or pose is None):
    sys.exit("Leave Isaac in Play with Go2 stopped in manual mode, then retry.")
if sys.argv[2] != "mapping" and (
        math.hypot(pose["x"] - spawn["position"][0], pose["y"] - spawn["position"][1]) > 0.30
        or abs(math.remainder(pose["yaw_rad"] - math.radians(spawn["yaw_deg"]), math.tau)) > 0.30):
    sys.exit("Go2 is away from its starting position. In Isaac, use Stop then Play, wait, and retry.")
PY

nodes=$(ros2 node list --no-daemon --spin-time 2 2>"$output/discovery.log")
if grep -Eq '^/(bt_navigator|controller_server|slam_toolbox|amcl)$' <<<"$nodes"; then
    echo 'An existing Nav2 or SLAM session is running. Stop it before using this menu.' >&2
    exit 1
fi
existing_relay=$(grep -cx /go2_relay_wsl <<<"$nodes" || true)
if [[ "$existing_relay" == 1 ]]; then
    echo 'Using the existing WSL relay.' | tee "$output/relay.log"
elif [[ "$existing_relay" == 0 ]]; then
    python3 tools/ros/wsl_go2_relay.py --windows-python "$GO2_WINDOWS_PYTHON" \
        --isaac-dir "$isaac_dir" >"$output/relay.log" 2>&1 9>&- &
    relay_pid=$!
else
    echo 'More than one WSL relay is running. Keep a single relay and retry.' >&2
    exit 1
fi
if [[ "$mode" == mapping ]]; then
    launch=(mapping)
    check=(check --seconds 2)
else
    launch=(navigation initial_pose:=spawn)
    check=(check --seconds 2 --drive-ready)
fi
bash tools/ros/go2_nav2.sh "${launch[@]}" >"$output/navigation.log" 2>&1 9>&- &
nav_pid=$!
echo 'Waiting for localization or SLAM and LiDAR...'
ready=0
processes=("$nav_pid")
if [[ -n $relay_pid ]]; then processes+=("$relay_pid"); fi
for _ in 1 2 3; do
    if ! kill -0 "${processes[@]}" 2>/dev/null; then break; fi
    if bash tools/ros/go2_nav2.sh "${check[@]}" >>"$output/readiness.log" 2>&1; then
        ready=1
        break
    fi
done
if [[ "$ready" != 1 ]] || ! kill -0 "${processes[@]}" 2>/dev/null; then
    echo "Nav2 did not become ready. See $output/navigation.log, relay.log and readiness.log." >&2
    exit 1
fi
clock_info=$(ros2 topic info /clock --no-daemon --spin-time 2)
if ! grep -qx 'Publisher count: 1' <<<"$clock_info"; then
    echo 'Expected one /clock publisher. Check for a second relay or rosbag.' >&2
    exit 1
fi
if [[ "$mode" == mapping ]]; then
    echo 'Drive with the keyboard in Isaac. Park, release the keys, then return here.'
    read -r -p 'Press Enter to save the map and exit. Ctrl+C cancels. ' _
    bash tools/ros/go2_nav2.sh save "$output/map"
    echo "Map saved at $output/map.yaml"
    exit 0
fi

set --
case "$mode" in
    nav) ;;
    cancel) set -- --cancel-after 2 ;;
    obstacle)
        kit nav2_inspection --arg action=obstacle_add --arg 'obstacle_xy=(-6.25,-0.20)' >>"$output/kit.log" 2>&1
        obstacle_added=1
        ;;
    *) exit 2 ;;
esac
echo "Running $mode. Esc in the Isaac viewport returns control to manual."
demo_started=1
bash tools/ros/go2_nav2.sh demo --timeout 240 --output "$output/nav2" "$@"
