#!/usr/bin/env bash
# Incremental build and test in a long-lived container, for agents and people iterating on a branch.
# The full `docker build --target test` stays the reference (CI); this script only saves time while editing.
#
#   scripts/dev.sh up    <name>              build the `base` image (cached) and start container lightnav-dev-<name>
#   scripts/dev.sh build <name> [pkg ...]    colcon build (only the given packages and what they need, or everything)
#   scripts/dev.sh test  <name> pkg [...]    colcon test of the given packages only, then print the result
#   scripts/dev.sh shell <name>              open a shell in the container
#   scripts/dev.sh down  <name>              remove the container and its build/install/log volumes
#
# The repository is bind-mounted read-write at /ws; build/, install/ and log/ are named volumes, so they survive
# between commands (on Linux, empty root-owned build/, install/ and log/ mount points appear in the tree; they are
# in .gitignore). No --network host: tests run on domain 44 inside the
# container only, so several branches can be tested on one machine at the same time.
set -euo pipefail

usage() { sed -n '2,12p' "$0" >&2; exit 2; }
[ "$#" -ge 2 ] || usage
command_name=$1
name=$2
shift 2
case "$name" in *[!a-zA-Z0-9_.-]*|'') echo "dev.sh: name must be [a-zA-Z0-9_.-]+" >&2; exit 2;; esac

repository_root=$(git rev-parse --show-toplevel)
container="lightnav-dev-${name}"
base_image="lightnav:dev-base"
cmake_arguments=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_MODULE_PATH=/usr/share/cmake/geographiclib)

in_container() { docker exec "$container" bash -c ". /opt/ros/jazzy/setup.sh && cd /ws && $1"; }
for package in "$@"; do
  case "$package" in *[!a-z0-9_]*|'') echo "dev.sh: invalid package name: $package" >&2; exit 2;; esac
done
package_arguments() { [ "$#" -eq 0 ] || printf -- '--packages-up-to %s' "$*"; }

case "$command_name" in
  up)
    docker build -q -f "$repository_root/docker/Dockerfile" --target base -t "$base_image" "$repository_root" >/dev/null
    if docker inspect "$container" >/dev/null 2>&1; then
      mounted=$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/ws"}}{{.Source}}{{end}}{{end}}' "$container")
      if [ "$mounted" != "$repository_root" ]; then
        echo "dev.sh: $container mounts $mounted, not $repository_root; run 'scripts/dev.sh down $name' first" >&2
        exit 1
      fi
      docker start "$container" >/dev/null
    else
      docker run -d --name "$container" --shm-size=1g \
        -v "$repository_root:/ws" \
        -v "${container}-build:/ws/build" -v "${container}-install:/ws/install" -v "${container}-log:/ws/log" \
        -e ROS_DOMAIN_ID=44 -e PYTHONPATH=/ws/tools/pytest_plugins -e PYTEST_PLUGINS=lightnav_test_environment \
        -e PYTEST_ADDOPTS="-o faulthandler_timeout=120 -p no:cacheprovider" -e PYTHONDONTWRITEBYTECODE=1 \
        --entrypoint sleep "$base_image" infinity >/dev/null
    fi
    echo "dev.sh: $container is up (repository $repository_root)"
    ;;
  build)
    in_container "colcon build --merge-install $(package_arguments "$@") --cmake-args ${cmake_arguments[*]}"
    ;;
  test)
    [ "$#" -ge 1 ] || { echo "dev.sh: give the packages to test" >&2; exit 2; }
    in_container "colcon build --merge-install --packages-up-to $* --cmake-args ${cmake_arguments[*]} \
      && . install/setup.sh && colcon test --merge-install --executor sequential --packages-select $* \
      --return-code-on-test-failure; status=\$?; for package in $*; do colcon test-result --verbose --test-result-base build/\$package; done; exit \$status"
    ;;
  shell)
    docker exec -it "$container" bash
    ;;
  down)
    docker rm -f "$container" >/dev/null 2>&1 || true
    docker volume rm "${container}-build" "${container}-install" "${container}-log" >/dev/null 2>&1 || true
    echo "dev.sh: $container removed"
    ;;
  *) usage ;;
esac
