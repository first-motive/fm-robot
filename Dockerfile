# fm-robot image — the robot layer, FROM the shared fm-docker base.
#
# Adds the ros2_control stack on top of the base's viz/description tooling, so
# this image can drive controllers as well as view a robot. Downstream images
# (fm-sim, fm-teleop) are FROM this one, so the control layer is shared rather
# than rebuilt. The entrypoint, WORKDIR, and viz/xacro tooling are inherited
# from the base — this layer only adds apt packages.
FROM ghcr.io/first-motive/fm-docker:humble

ARG DEBIAN_FRONTEND=noninteractive

# ros2_control: controller_manager + the joint-trajectory + forward controllers
# and broadcasters that fm_control configures. All on the Humble apt mirror for
# both arm64 and amd64, so no source builds.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-joint-trajectory-controller \
    && rm -rf /var/lib/apt/lists/*

# The joint-state publishers view_robot.launch.py dispatches: headless
# joint_state_publisher on the foxglove path, joint_state_publisher_gui on the rviz
# path (use_jsp_gui defaults to auto and follows the viewer).
#
# fm_description already declares both as exec_depend, and every rosdep-driven path
# — CI's colcon build, the native pixi env — installs them from that declaration.
# This image does not run rosdep; it installs a hand-written list, so a declared
# dependency reaches the container only if someone also adds it here. That gap made
# `run.sh --container --viewer rviz` die on "package 'joint_state_publisher_gui' not
# found", which fm-ros2's ci-smoke caught the first time it was allowed to run.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-joint-state-publisher \
      ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*
