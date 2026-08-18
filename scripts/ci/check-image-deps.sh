#!/usr/bin/env bash
# Assert every dependency this repo's packages declare is present in the built image.
#
#   ./scripts/ci/check-image-deps.sh fm-robot:check
#
# The images install a hand-written apt list and never run rosdep, so a `<depend>` in
# a package.xml reaches a container only if someone separately remembers a Dockerfile
# line. The two lists are copies of the same facts and nothing compared them, which is
# how `joint_state_publisher_gui` went missing and broke the container rviz path
# entirely (fm-robot#35). This is the comparison.
#
# Each declared key is classified on the host with rosdep, then checked inside the
# image in one container invocation:
#
#   resolves to ros-humble-*  -> a ROS package  -> `ros2 pkg prefix` finds it
#   resolves to anything else -> a system dep   -> `dpkg -s` finds it
#   resolves to nothing       -> built in this workspace -> `ros2 pkg prefix` finds it
#
# `ros2 pkg prefix` is the right question for a ROS package: it asks what a launch
# file asks — can this be found at runtime — and is satisfied by an apt install and a
# workspace build alike.
#
# Not errexit: every dependency is checked so one run lists everything missing.
set -uo pipefail

usage() {
  cat <<'EOF'
check-image-deps.sh — assert declared dependencies exist in the image

Usage: ./scripts/ci/check-image-deps.sh <image> [-h]

  <image>      docker image to check, e.g. fm-robot:check
  -h, --help   show this help
EOF
}

# A name no package can plausibly have. The check runs it through the same lookup as
# a real dependency and requires it to be absent: a lookup that answers "present" for
# everything is a check that cannot fail, which is the thing this file exists to
# remove. If this canary ever passes, the check is broken, not the image.
CANARY="fm_this_package_does_not_exist"

main() {
  case "${1:-}" in
    -h|--help) usage; return 0 ;;
  esac
  local image="${1:?usage: check-image-deps.sh <image>}"

  local ROOT
  ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  cd "$ROOT" || return 1

  # Every dependency our packages declare, minus the packages we are ourselves.
  local declared own deps
  declared="$(grep -hoE '<(exec_depend|depend|build_depend)>[^<]+' ./*/package.xml |
              sed 's/.*>//' | sort -u)"
  own="$(grep -hoE '<name>[^<]+' ./*/package.xml | sed 's/.*>//' | sort -u)"
  deps="$(comm -23 <(echo "$declared") <(echo "$own"))"
  echo "==> ${ROOT##*/}: $(echo "$deps" | grep -c .) declared dependencies to check"

  # An uninitialised rosdep answers "" for every key, which would classify system
  # packages as workspace packages and check them the wrong way — a whole run of
  # meaningless results. Prove the database works on a key that must resolve before
  # trusting any answer from it.
  local probe
  probe="$(rosdep resolve rclcpp 2>/dev/null | grep -v '^#' | tr -d ' \n')"
  if [ -z "$probe" ]; then
    echo "error: rosdep cannot resolve rclcpp — its database is missing or stale." >&2
    echo "       run: sudo rosdep init && rosdep update" >&2
    return 1
  fi
  # rosdep answers for the host's OS, and the image is Debian. On macOS the same key
  # resolves to `ros/humble/rclcpp` rather than the apt name, and every system
  # dependency resolves to nothing — which would classify them as ROS packages and
  # report a page of failures that are not real. Refuse rather than mislead.
  case "$probe" in
    ros-*) ;;
    *) echo "error: rosdep is resolving for this host, not for the image's Debian." >&2
       echo "       got '$probe' for rclcpp, expected an apt name like ros-humble-rclcpp." >&2
       echo "       Run this on Linux, or inside a ros:\${ROS_DISTRO} container." >&2
       return 1 ;;
  esac

  # Classify on the host: rosdep knows which keys are ROS packages and which are
  # system packages, so the script needs no hand-maintained list of exceptions —
  # a hand-maintained list is the failure mode being fixed here.
  local ros_pkgs="" apt_pkgs="" dep resolved
  for dep in $deps $CANARY; do
    resolved="$(rosdep resolve "$dep" 2>/dev/null | grep -v '^#' | tr '\n' ' ')"
    case "$resolved" in
      "")                ros_pkgs="$ros_pkgs $dep" ;;   # built in this workspace
      ros-humble-*)      ros_pkgs="$ros_pkgs $dep" ;;
      *)                 apt_pkgs="$apt_pkgs $resolved" ;;
    esac
  done

  docker run --rm \
    -e ROS_PKGS="$ros_pkgs" -e APT_PKGS="$apt_pkgs" -e CANARY="$CANARY" \
    "$image" bash -lc '
      set -uo pipefail
      source "/opt/ros/${ROS_DISTRO}/setup.bash" 2>/dev/null || true
      [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
      fails=0
      for p in $ROS_PKGS; do
        [ "$p" = "$CANARY" ] && continue
        if ros2 pkg prefix "$p" >/dev/null 2>&1; then
          echo "PASS: ros package $p"
        else
          echo "FAIL: $p is declared in a package.xml but not installed in the image"
          fails=$((fails + 1))
        fi
      done
      for p in $APT_PKGS; do
        if dpkg -s "$p" >/dev/null 2>&1; then
          echo "PASS: system package $p"
        else
          echo "FAIL: $p is declared in a package.xml but not installed in the image"
          fails=$((fails + 1))
        fi
      done
      if ros2 pkg prefix "$CANARY" >/dev/null 2>&1; then
        echo "FAIL: canary $CANARY resolved — this check cannot detect a missing package"
        fails=$((fails + 1))
      else
        echo "PASS: canary absent, so the check can fail"
      fi
      echo "==> image deps: ${fails} failure(s)"
      [ "$fails" -eq 0 ]
    '
}

main "$@"
