# Shared by login shells, interactive bash, and the container entrypoint.
# Do not enable nounset here; this file is sourced from .bashrc.
if [ -f /opt/ros/jazzy/setup.bash ]; then
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    set +u
fi

export AIRBOT_WORKSPACE="${AIRBOT_WORKSPACE:-/home/ubuntu/workspace}"
export VIRTUAL_ENV=/opt/airbot-venv
case ":${PATH}:" in
    *":${VIRTUAL_ENV}/bin:"*) ;;
    *) export PATH="${VIRTUAL_ENV}/bin:${PATH}" ;;
esac

AIRDC="${AIRBOT_WORKSPACE}/Airbot_play/AIRBOT-Data-Collection-develop"
if [ -d "${AIRDC}" ]; then
    case ":${PYTHONPATH:-}:" in
        *":${AIRDC}:"*) ;;
        *) export PYTHONPATH="${AIRDC}${PYTHONPATH:+:${PYTHONPATH}}" ;;
    esac
fi

ROS2_WS="${AIRBOT_WORKSPACE}/ros2_ws"
if [ -f "${ROS2_WS}/install/setup.bash" ]; then
    set +u
    # shellcheck source=/dev/null
    source "${ROS2_WS}/install/setup.bash"
    set +u
fi
