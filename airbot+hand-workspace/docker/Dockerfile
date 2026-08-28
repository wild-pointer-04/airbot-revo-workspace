FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    AIRBOT_WORKSPACE=/home/ubuntu/airbot+hand-workspace \
    AIRBOT_VENV=/opt/airbot-venv

RUN apt-get update && apt-get install -y --no-install-recommends \
        can-utils docker.io iproute2 libspdlog-dev net-tools \
        python3-pip python3-venv sudo udev usbutils \
    && rm -rf /var/lib/apt/lists/*

ARG USER=dt
ARG UID=1000
ARG GID=1000

RUN set -eux; \
    if id -u ubuntu >/dev/null 2>&1; then \
        usermod -l "${USER}" ubuntu; \
        groupmod -n "${USER}" ubuntu; \
        usermod -d "/home/${USER}" -m "${USER}"; \
    elif ! id -u "${USER}" >/dev/null 2>&1; then \
        groupadd --gid "${GID}" "${USER}"; \
        useradd --uid "${UID}" --gid "${GID}" -m -s /bin/bash "${USER}"; \
    fi; \
    usermod -aG sudo,dialout,video "${USER}"; \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER}"; \
    chmod 0440 "/etc/sudoers.d/${USER}"

COPY Airbot_play/5.1.6/airbot_cpp-5.1.6-x86_64-noble.deb /tmp/airbot_cpp.deb
COPY Airbot_play/airbot-configure_5.1.6-1_all.deb /tmp/airbot-configure.deb
COPY Airbot_play/5.1.6/airbot_py-5.1.6-py3-none-any.whl /tmp/airbot_py-5.1.6-py3-none-any.whl
COPY Airbot_play/AIRBOT-Data-Collection-develop /tmp/airdc

# The package postinst manages host kernel modules, udev and systemd. A
# container installs the payload only and leaves host device setup untouched.
RUN apt-get update \
    && apt-get install -y --no-install-recommends /tmp/airbot_cpp.deb \
    && dpkg-deb -x /tmp/airbot-configure.deb / \
    && python3 -m venv "${AIRBOT_VENV}" \
    && "${AIRBOT_VENV}/bin/pip" install --no-cache-dir --upgrade pip \
    && "${AIRBOT_VENV}/bin/pip" install --no-cache-dir /tmp/airbot_py-5.1.6-py3-none-any.whl \
    && "${AIRBOT_VENV}/bin/pip" install --no-cache-dir numpy pyyaml pydantic pydantic-settings omegaconf hydra-zen \
    && "${AIRBOT_VENV}/bin/pip" install --no-cache-dir --no-deps -e /tmp/airdc \
    && rm -rf /var/lib/apt/lists/* /tmp/airbot_cpp.deb /tmp/airbot-configure.deb /tmp/airbot_py-5.1.6-py3-none-any.whl

COPY docker/entrypoint.sh docker/airbot_up.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/airbot_up.sh \
    && mkdir -p "${AIRBOT_WORKSPACE}" \
    && chown -R "${UID}:${GID}" /home/ubuntu "${AIRBOT_VENV}" /tmp/airdc

ENV PATH="${AIRBOT_VENV}/bin:${PATH}" \
    HOME=/home/dt \
    USER=dt

USER ${USER}
WORKDIR ${AIRBOT_WORKSPACE}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
