# syntax=docker/dockerfile:1
# Build context is the repository root:
#   docker build -f docker/Dockerfile -t lightnav:local .                  (runtime, the last stage)
#   docker build -f docker/Dockerfile --target test -t lightnav:test .     (CI only)
# Stages: base -> build -> test; base -> runtime (copies only /ws/install from build).

# Same base as tsukuchalle_drivers/ros2/Dockerfile. Digest taken 2026-11-06 (docker buildx imagetools inspect ros:jazzy-ros-base-noble).
FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617 AS base

# Fast DDS is configured by /fastdds_profiles.xml (a copy of tsukuchalle_drivers/ros2/dds/): SHM with a 16 MiB
# segment plus UDPv4 on 127.0.0.1 only, no multicast. The transports and the discovery range must come from the
# XML and ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT, not from LOCALHOST (LOCALHOST makes rmw_fastrtps add a
# default 512 KiB SHM transport after reading the XML). FASTDDS_BUILTIN_TRANSPORTS is not set on purpose.
# Containers need `ipc: host` and `network_mode: host`. Shared-memory files belong to the uid that made them, so every
# ROS process on the host must run as the same uid; /dds_check.sh stops a mismatch (TSUKUCHALLE_UID).
# ROS_LOG_DIR / ROS_HOME are under /tmp because a non-root uid cannot write to / or a missing HOME.
ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
    FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_profiles.xml \
    ROS_LOG_DIR=/tmp/ros_log \
    ROS_HOME=/tmp/ros_home

# Dependencies are listed here instead of `rosdep install`, so the apt layer stays cached when src/ changes.
# The `test` stage runs `rosdep check` to catch a package.xml dependency missing from this list.
# The ROS apt repository drops old versions, so the ROS packages are not pinned.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libeigen3-dev \
        libgeographiclib-dev \
        libgtest-dev \
        libtbb-dev \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pytest \
        python3-yaml \
        ros-jazzy-gtsam \
        ros-jazzy-tf2-eigen \
        ros-jazzy-tf2-ros-py \
    && rm -rf /var/lib/apt/lists/*

# Last in base, so editing them does not rebuild the apt layer above.
COPY --chmod=0644 docker/dds/fastdds_profiles.xml /fastdds_profiles.xml
COPY --chmod=0755 docker/dds/dds_check.sh /dds_check.sh


FROM base AS build
WORKDIR /ws
COPY src src
# BUILD_TESTING stays ON (default): the gtest executables are small and the `test` stage reuses this build.
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release


# CI only. Tests use the production XML (inherited from base), not LOCALHOST like `pixi run test`.
# Domain 44 and the plugin are the same as the `test` task of pixi.toml; the plugin aborts unless the domain is 44.
FROM build AS test
COPY tools tools
ENV ROS_DOMAIN_ID=44 \
    PYTHONPATH=/ws/tools/pytest_plugins \
    PYTEST_PLUGINS=lightnav_test_environment \
    PYTEST_ADDOPTS="-o faulthandler_timeout=120"
ENTRYPOINT ["/dds_check.sh"]
# `colcon test` stops before `colcon test-result` on failure, so print the result in every case.
CMD ["bash", "-c", ". /ws/install/setup.bash && colcon test --merge-install --executor sequential --return-code-on-test-failure; test_status=$?; colcon test-result --verbose; exit $test_status"]


FROM base AS runtime
WORKDIR /ws
COPY --from=build /ws/install /ws/install
COPY docker/docker-entrypoint.sh /ws/docker-entrypoint.sh
ENTRYPOINT ["/dds_check.sh", "/ws/docker-entrypoint.sh"]
# PR 1b replaces this with `ros2 launch lightnav_bringup robot.launch.py`.
CMD ["bash"]
