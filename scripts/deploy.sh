#!/usr/bin/env bash
# Builds the lightnav image on the build machine and installs it on the robot PC, through this machine
# (the build machine cannot reach the robot directly). Run from a clean checkout of this repository.
#
#   LIGHTNAV_BUILD_HOST=<ssh destination> LIGHTNAV_ROBOT_HOST=<ssh destination> scripts/deploy.sh [git-ref]
#
# git-ref defaults to origin/develop and must be pushed. The destinations are only read from the environment:
# no host name or address belongs in this repository. This script never starts a container on the robot.
set -euo pipefail

ROBOT_PROJECT_PATH='~/lightnav'
MINIMUM_FREE_GIB=3
EXPECTED_ARCHITECTURE=x86_64

usage() {
  echo "usage: LIGHTNAV_BUILD_HOST=<ssh destination> LIGHTNAV_ROBOT_HOST=<ssh destination> $0 [git-ref]" >&2
  echo "  git-ref: a pushed commit (default origin/develop)" >&2
}

if [ -z "${LIGHTNAV_BUILD_HOST:-}" ] || [ -z "${LIGHTNAV_ROBOT_HOST:-}" ] || [ "$#" -gt 1 ]; then
  usage
  exit 2
fi
git_ref="${1:-origin/develop}"

repository_root=$(git rev-parse --show-toplevel)
cd "$repository_root"
if [ -n "$(git status --porcelain)" ]; then
  echo "deploy: the working tree has uncommitted changes; commit or stash them first" >&2
  exit 1
fi
# Without this, a stale local origin/develop would deploy an old commit.
git fetch --quiet origin
commit_sha=$(git rev-parse --verify "${git_ref}^{commit}")
short_sha=$(git rev-parse --short=7 "$commit_sha")
repository_url=$(git remote get-url origin)
image_tag="lightnav:${short_sha}"

ssh_build() { ssh -o BatchMode=yes "$LIGHTNAV_BUILD_HOST" "$@"; }
ssh_robot() { ssh -o BatchMode=yes "$LIGHTNAV_ROBOT_HOST" "$@"; }

echo "deploy: ${git_ref} = ${short_sha}"

# 1. The architectures must match, and the robot needs room for the image.
build_architecture=$(ssh_build uname -m)
robot_architecture=$(ssh_robot uname -m)
if [ "$build_architecture" != "$EXPECTED_ARCHITECTURE" ] || [ "$robot_architecture" != "$EXPECTED_ARCHITECTURE" ]; then
  echo "deploy: expected ${EXPECTED_ARCHITECTURE} on both machines, got build=${build_architecture} robot=${robot_architecture}" >&2
  exit 1
fi
free_gib=$(ssh_robot 'df --output=avail -BG "$(docker info --format "{{.DockerRootDir}}")" | tail -n 1 | tr -dc 0-9')
if [ "$free_gib" -lt "$MINIMUM_FREE_GIB" ]; then
  echo "deploy: the Docker storage of the robot has ${free_gib} GiB free; ${MINIMUM_FREE_GIB} GiB are needed" >&2
  exit 1
fi

# 2. Files for the robot. config/robot.yaml may have been tuned on site: show the difference and stop instead of overwriting.
staging_directory=$(mktemp -d)
trap 'rm -rf "$staging_directory"' EXIT
git archive "$commit_sha" docker/compose.yaml docker/config docker/.env.example | tar -x -C "$staging_directory"
if ssh_robot "test -e ${ROBOT_PROJECT_PATH}/config/robot.yaml"; then
  ssh_robot "cat ${ROBOT_PROJECT_PATH}/config/robot.yaml" > "$staging_directory/robot.yaml.on-robot"
  if ! diff -u "$staging_directory/robot.yaml.on-robot" "$staging_directory/docker/config/robot.yaml"; then
    echo "deploy: config/robot.yaml on the robot differs from ${short_sha} (diff above: - robot, + repository)." >&2
    echo "deploy: put the value you want into docker/config/robot.yaml through a PR, or remove the file on the robot, then run again." >&2
    exit 1
  fi
fi

# 3. Build on the build machine, in the clone that only this script uses (do not use it by hand: it is checked out detached).
ssh_build bash -s -- "$repository_url" "$commit_sha" "$image_tag" <<'REMOTE_SCRIPT'
set -euo pipefail
repository_url=$1
commit_sha=$2
image_tag=$3
clone_path=$HOME/deploy/lightweight-nav-stack
mkdir -p "$(dirname "$clone_path")"
[ -d "$clone_path/.git" ] || git clone "$repository_url" "$clone_path"
cd "$clone_path"
git fetch origin
if [ -n "$(git status --porcelain)" ]; then
  echo "deploy: ${clone_path} has uncommitted changes on the build machine; it is for this script only" >&2
  exit 1
fi
git checkout --detach "$commit_sha"
docker build -f docker/Dockerfile -t "$image_tag" .
REMOTE_SCRIPT

# 4. Image: build machine -> this machine -> robot. `latest` moves only after the load succeeded.
ssh_build "docker save ${image_tag} | gzip" | ssh_robot 'gunzip | docker load'
ssh_robot "docker tag ${image_tag} lightnav:latest"

# 5. compose.yaml, config/, .env.example. .env is never overwritten; it is created from .env.example only when missing.
tar -C "$staging_directory/docker" -c compose.yaml config .env.example | ssh_robot "mkdir -p ${ROBOT_PROJECT_PATH}/data && tar -x -C ${ROBOT_PROJECT_PATH}"
if ssh_robot "test -e ${ROBOT_PROJECT_PATH}/.env"; then
  env_note=".env: kept as it is."
else
  ssh_robot "cp ${ROBOT_PROJECT_PATH}/.env.example ${ROBOT_PROJECT_PATH}/.env"
  env_note=".env: created from .env.example. CHECK TSUKUCHALLE_UID / TSUKUCHALLE_GID before starting (they must match the other containers)."
fi

cat <<EOF_MESSAGE

deploy: ${image_tag} (and latest) installed in ${ROBOT_PROJECT_PATH} on the robot. ${env_note}
Nothing was started. On the robot, with a person's approval each time:
  cd ${ROBOT_PROJECT_PATH}
  docker compose up -d nav                          # starts safety_gate_node (/cmd_vel); drive's motors stay off until the gamepad Start
  LIGHTNAV_SAFETY_GATE=false docker compose up -d nav   # teaching drive only (no /cmd_vel publisher from this stack)
Before Start: confirm mission is IDLE (ros2 action list / /lightnav/follow_status).
Stop: docker compose exec nav bash -lc 'ros2 lifecycle set /safety_gate_node deactivate' && docker compose stop nav
EOF_MESSAGE
