#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
export PATH="/opt/airbot-venv/bin:${PATH}"
export VIRTUAL_ENV=/opt/airbot-venv

cd /home/ubuntu/airbot+hand-workspace
exec "$@"
