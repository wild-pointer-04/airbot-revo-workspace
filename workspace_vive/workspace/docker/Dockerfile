FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    AIRBOT_WORKSPACE=/home/ubuntu/workspace \
    AIRBOT_VENV=/opt/airbot-venv \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        can-utils \
        cmake \
        docker.io \
        iproute2 \
        kmod \
        libeigen3-dev \
        libgl1 \
        libspdlog-dev \
        libturbojpeg \
        libusb-1.0-0 \
        net-tools \
        procps \
        python3-colcon-common-extensions \
        python3-pip \
        python3-venv \
        python3-yaml \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-xacro \
        sudo \
        udev \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

ARG USER=dt
ARG UID=1000
ARG GID=1000
ARG DOCKER_GID=126

RUN set -eux; \
    if id -u ubuntu >/dev/null 2>&1; then \
        usermod -l "${USER}" ubuntu; \
        groupmod -n "${USER}" ubuntu || true; \
        usermod -d "/home/${USER}" -m "${USER}"; \
    elif ! id -u "${USER}" >/dev/null 2>&1; then \
        groupadd --gid "${GID}" "${USER}"; \
        useradd --uid "${UID}" --gid "${GID}" -m -s /bin/bash "${USER}"; \
    fi; \
    if getent group docker >/dev/null; then \
        groupmod -g "${DOCKER_GID}" docker || true; \
    else \
        groupadd --gid "${DOCKER_GID}" docker; \
    fi; \
    usermod -aG sudo,dialout,video,plugdev,docker "${USER}" || \
        usermod -aG sudo,dialout,video,docker "${USER}"; \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER}"; \
    chmod 0440 "/etc/sudoers.d/${USER}"

COPY Airbot_play/5.1.6/airbot_cpp-5.1.6-x86_64-noble.deb /tmp/airbot_cpp.deb
COPY Airbot_play/airbot-configure_5.1.6-1_all.deb /tmp/airbot-configure.deb
COPY Airbot_play/5.1.6/airbot_py-5.1.6-py3-none-any.whl /tmp/airbot_py-5.1.6-py3-none-any.whl
COPY docker/requirements.txt /tmp/requirements.txt

# airbot-configure postinst talks to host udev/systemd. Extract payload only.
RUN apt-get update \
    && apt-get install -y --no-install-recommends /tmp/airbot_cpp.deb \
    && dpkg-deb -x /tmp/airbot-configure.deb / \
    && python3 -m venv --system-site-packages "${AIRBOT_VENV}" \
    && "${AIRBOT_VENV}/bin/pip" install --no-cache-dir --upgrade pip \
    && "${AIRBOT_VENV}/bin/pip" install --no-cache-dir /tmp/airbot_py-5.1.6-py3-none-any.whl \
    && "${AIRBOT_VENV}/bin/pip" install --no-cache-dir -r /tmp/requirements.txt \
    && rm -rf /var/lib/apt/lists/* \
        /tmp/airbot_cpp.deb \
        /tmp/airbot-configure.deb \
        /tmp/airbot_py-5.1.6-py3-none-any.whl \
        /tmp/requirements.txt

COPY docker/airbot_env.sh /opt/airbot_env.sh
COPY docker/entrypoint.sh docker/airbot_up.sh docker/can_up.sh /usr/local/bin/
RUN chmod 644 /opt/airbot_env.sh \
    && chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/airbot_up.sh /usr/local/bin/can_up.sh \
    && mkdir -p "${AIRBOT_WORKSPACE}" \
    && printf '\n# AIRBOT workspace\n[ -f /opt/airbot_env.sh ] && . /opt/airbot_env.sh\n' >> "/home/${USER}/.bashrc" \
    && chown -R "${UID}:${GID}" "/home/${USER}" "${AIRBOT_VENV}" "${AIRBOT_WORKSPACE}"

ENV PATH="${AIRBOT_VENV}/bin:${PATH}" \
    HOME=/home/dt \
    USER=dt \
    VIRTUAL_ENV=/opt/airbot-venv

USER ${USER}
WORKDIR ${AIRBOT_WORKSPACE}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
