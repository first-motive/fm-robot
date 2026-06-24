#!/usr/bin/env bash
# Standalone front door for fm-robot. Builds the workspace and launches a robot
# description view — Foxglove Studio renders it at ws://localhost:8765.
#
# The host OS picks the path (override with --native / --container):
#   Linux  -> native:    build + launch directly on the host (ROS2 Humble + the
#                        ros2_control deps must be installed)
#   Darwin -> container: build the fm-robot image, bring it up via the fm-docker
#                        compose overlays, build + launch inside it (OrbStack)
#
#   ./run.sh                       # auto-detect, default robot (g1_d)
#   ./run.sh --robot so101         # pick a robot (hyphen or underscore form)
#   ./run.sh --native              # force the host path (Linux)
#   ./run.sh --container           # force the container path (macOS / OrbStack)
#   ./run.sh --robot openarm use_rviz:=true   # extra args pass through to launch
set -euo pipefail

cd "$(dirname "$0")"

# --- Per-repo config (downstream repos retune these three) --------------------
IMAGE=fm-robot:humble                              # local image tag for the container path
LAUNCH=(ros2 launch fm_description view_robot.launch.py)  # what `run.sh` launches
VALID_ROBOTS=(g1_d so101 openarm)                  # robots this repo can launch
# -----------------------------------------------------------------------------

ROBOT=g1_d
MODE=""                  # "" = auto-detect; else native | container
PASSTHROUGH=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot)     ROBOT="$2"; shift 2 ;;
    --robot=*)   ROBOT="${1#--robot=}"; shift ;;
    --native)    MODE=native; shift ;;
    --container) MODE=container; shift ;;
    *)           PASSTHROUGH+=("$1"); shift ;;
  esac
done

# Normalize hyphen -> underscore (g1-d -> g1_d) and validate.
ROBOT="${ROBOT//-/_}"
ok=false
for r in "${VALID_ROBOTS[@]}"; do [[ "$ROBOT" == "$r" ]] && ok=true && break; done
if [[ "$ok" != true ]]; then
  echo "error: unknown robot '$ROBOT' — valid: ${VALID_ROBOTS[*]}" >&2
  exit 1
fi

# Auto-detect the path from the host OS when not forced by a flag.
if [[ -z "$MODE" ]]; then
  case "$(uname -s)" in
    Linux)  MODE=native ;;
    Darwin) MODE=container ;;
    *) echo "error: unsupported host '$(uname -s)' — pass --native or --container" >&2; exit 1 ;;
  esac
fi

LAUNCH+=("robot:=$ROBOT" ${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"})

if [[ "$MODE" == native ]]; then
  # Host path: pull externals once, build in place, launch on the host.
  set +u  # ROS setup scripts reference unbound vars; nounset would abort the source
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  set -u
  if [[ ! -d external ]]; then
    vcs import < fm-robot.repos
  fi
  rosdep install --from-paths . external --ignore-src -y -r 2>/dev/null || true
  colcon build --symlink-install
  set +u
  source install/setup.bash
  set -u
  echo ">> launching $ROBOT on the host — Foxglove Studio: ws://localhost:8765"
  exec "${LAUNCH[@]}"
fi

# Container path: build the local image, bring it up, build + launch inside it.
# The fm-docker compose overlays live in docker/, imported via fm-robot.repos —
# pull them on first run so a fresh clone works with no manual setup.
if [[ ! -d docker ]]; then
  vcs import < fm-robot.repos
fi
COMPOSE=(docker compose -f docker/compose.yaml -f docker/compose.macos.yaml)
export FM_IMAGE="$IMAGE"
export FM_WS="$PWD"

echo ">> building $IMAGE (FROM the fm-docker base)"
docker build -t "$IMAGE" .
echo ">> bringing the container up (idempotent)"
"${COMPOSE[@]}" up -d
echo ">> building the workspace inside the container"
"${COMPOSE[@]}" exec fm /ros_entrypoint.sh colcon build --symlink-install
echo ">> launching $ROBOT — Foxglove Studio: ws://localhost:8765"
echo ">> tear down with: ${COMPOSE[*]} down"
# `exec` skips the image ENTRYPOINT, so route through it to source ROS + overlay.
exec "${COMPOSE[@]}" exec fm /ros_entrypoint.sh "${LAUNCH[@]}"
