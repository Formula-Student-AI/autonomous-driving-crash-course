# Dev container for the Autonomous Driving Crash Course.
#
# Starts from the official ROS 2 Humble base image (NOT ros-humble-desktop: we
# visualise with Foxglove, so rviz/Qt tooling is not needed) and installs every
# system dependency the simulator declares.
#
# The simulator itself is NOT built here. It is built once into sim/install when
# the container is first created (see devcontainer.json's postCreateCommand), so
# there is only ever one copy of the source: the one in this repository.

FROM ros:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# --- base tooling -----------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        git \
        curl \
        nano \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# --- simulator dependencies -------------------------------------------------
# Resolved from each package.xml rather than hand-listed, so adding a dependency
# to a package is enough; nothing in this file needs editing. The sources are
# copied in only to resolve the dependencies, then deleted.
#
# NOTE: this means the installed dependencies are baked into the image. If you
# add a dependency to a package.xml you must rebuild the container
# (Dev Containers: Rebuild Container), not just re-run colcon build.
COPY sim/src /tmp/sim-src
RUN apt-get update \
    && (rosdep update --rosdistro "$ROS_DISTRO" || true) \
    && rosdep install --from-paths /tmp/sim-src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* /tmp/sim-src

# --- non-root user ----------------------------------------------------------
# UID/GID 1000 matches the typical host user so files created in the container
# are owned by you on the host.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000
RUN if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# --- simulator scratch directory --------------------------------------------
# The simulator reads $EUFS_MASTER at startup and writes its logs and
# data/current/setup.json beneath it. Maps and robot models come from the
# installed packages, so this only needs to be a writable directory.
ENV EUFS_MASTER=/home/${USERNAME}/eufs-master
RUN mkdir -p "${EUFS_MASTER}" && chown -R ${USER_UID}:${USER_GID} "${EUFS_MASTER}"

# --- shell setup ------------------------------------------------------------
# Every interactive shell gets ROS and, once it has been built, the simulator.
# Students still source their own exercise workspace by hand: that is part of
# Module 1.
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
    && echo '[ -n "${FSAI_WS:-}" ] && [ -f "$FSAI_WS/sim/install/setup.bash" ] && source "$FSAI_WS/sim/install/setup.bash"' >> /etc/bash.bashrc

EXPOSE 8765

USER ${USERNAME}
