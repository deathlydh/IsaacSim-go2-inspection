#!/usr/bin/env bash
# Invoke with bash in Ubuntu-22.04 WSL. One launch per ROS domain.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
# Use Fast DDS UDP for these WSL processes.
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-"$PWD/tools/ros/go2_fastdds_udp.xml"}
case "${1:-}" in
    mapping|navigation)
        mode=$1
        shift
        exec flock --nonblock --no-fork "/tmp/go2-nav2-${ROS_DOMAIN_ID}.lock" \
            ros2 launch tools/ros/go2_nav2.launch.py "mode:=$mode" "$@"
        ;;
    save)
        if [[ $# != 2 || -e "$2.yaml" || -e "$2.pgm" ]]; then
            echo 'Usage: go2_nav2.sh save NEW_MAP_PREFIX' >&2
            exit 2
        fi
        mkdir -p "$(dirname "$2")"
        exec ros2 run nav2_map_server map_saver_cli -f "$2" --fmt pgm --free 0.196 --occ 0.65 \
            --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
        ;;
    demo|check|calibrate)
        exec python3 tools/ros/go2_nav2_run.py "$@"
        ;;
    *)
        echo 'Usage: go2_nav2.sh {mapping|navigation|save|demo|check|calibrate}' >&2
        exit 2
        ;;
esac
