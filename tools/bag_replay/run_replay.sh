#!/usr/bin/env bash
# Replays a rosbag through localization_node in the `replay` Docker image (run it on the build machine).
#
#   tools/bag_replay/run_replay.sh --bag ~/bags/drive-xxx --out ~/work/run1 [--build] [--image TAG] \
#       [replay_session.py options: --mode --rate --start-offset --duration --foxglove --foxglove-port ...]
#
# --foxglove publishes the port (default 8765) of the container on --foxglove-bind (default 0.0.0.0: every interface of the
# host; use 127.0.0.1 with an SSH tunnel). The bridge has no authentication.
# The container has its own network namespace (no --network host) and ROS_DOMAIN_ID=61.
set -euo pipefail

image="lightnav:replay"
bag_directory=""
out_directory=""
should_build=false
foxglove_port=8765
foxglove_bind_address=0.0.0.0
should_publish_port=false
session_arguments=()
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --bag) bag_directory="$2"; shift 2 ;;
    --out) out_directory="$2"; shift 2 ;;
    --image) image="$2"; shift 2 ;;
    --build) should_build=true; shift ;;
    --foxglove) should_publish_port=true; session_arguments+=("$1"); shift ;;
    --foxglove-bind) foxglove_bind_address="$2"; shift 2 ;;
    --foxglove-port) foxglove_port="$2"; session_arguments+=("$1" "$2"); shift 2 ;;
    *) session_arguments+=("$1"); shift ;;
  esac
done

[ -d "$bag_directory" ] || { echo "run_replay.sh: --bag must be a rosbag2 directory" >&2; exit 2; }
[ -n "$out_directory" ] || { echo "run_replay.sh: --out is required" >&2; exit 2; }
mkdir -p "$out_directory"

if [ "$should_build" = true ] || ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build -f "$repository_root/docker/Dockerfile" --target replay -t "$image" "$repository_root"
fi

port_arguments=()
if [ "$should_publish_port" = true ]; then
  port_arguments=(-p "${foxglove_bind_address}:${foxglove_port}:${foxglove_port}")
fi

# --user: the files in --out belong to the caller. MPLCONFIGDIR: matplotlib needs a writable directory.
exec docker run --rm --shm-size=1g --user "$(id -u):$(id -g)" -e MPLCONFIGDIR=/tmp/matplotlib \
  -v "$(cd "$bag_directory" && pwd)":/bag:ro -v "$(cd "$out_directory" && pwd)":/out \
  "${port_arguments[@]}" "$image" \
  python3 /ws/tools/bag_replay/replay_session.py --bag /bag --out /out "${session_arguments[@]}"
