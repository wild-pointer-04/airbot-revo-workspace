FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dt
ARG USER_UID=1000
ARG USER_GID=1000

ENV TZ=Asia/Shanghai \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=100 \
    QT_X11_NO_MITSHM=1

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    git-lfs \
    curl \
    wget \
    unzip \
    udev \
    usbutils \
    build-essential \
    cmake \
    gdb \
    libeigen3-dev \
    libgl1 \
    libglx-mesa0 \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    libudev-dev \
    python3-pip \
    python3-serial \
    python3-venv \
    python3-yaml \
    python3-tk \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-eigen3-cmake-module \
    ros-humble-pinocchio \
    ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    libncurses-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
        "empy==3.3.4" \
        catkin_pkg \
        rospkg \
        lark \
        numpy \
        pyyaml \
        setuptools \
        typeguard \
        typing_extensions \
        jinja2

# MANUS USB dongle / glove (vendor 3325)
RUN printf '%s\n' \
    '# HIDAPI/libusb' \
    'SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"' \
    '# HIDAPI/hidraw' \
    'KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"' \
    > /etc/udev/rules.d/99-manus.rules

RUN if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid "${USER_GID}" "${USERNAME}"; fi \
    && if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
         useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
       fi \
    && for grp in sudo dialout plugdev video; do \
         getent group "${grp}" >/dev/null && usermod -aG "${grp}" "${USERNAME}" || true; \
       done \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && mkdir -p "/home/${USERNAME}/manus-Revo2-Revo3" \
    && chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}"

COPY --chown=${USERNAME}:${USERNAME} . /home/dt/manus-Revo2-Revo3

USER ${USERNAME}
WORKDIR /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting

RUN bash -lc 'source /opt/ros/humble/setup.bash && \
    python3 -m colcon build --symlink-install --packages-select \
      manus_ros2_msgs hex_glove_driver manus_ros2 manus_revo2_retarget'

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /home/dt/manus-Revo2-Revo3

CMD ["bash"]
