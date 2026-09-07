# middle_interface

PICO OpenXR JSON → normalize / head-base / incremental teleop / safety → `RobotAdapter`.

Does **not** import PICO Unity SDK. Does **not** connect a real arm by default (`robot.adapter: dummy`).

Quaternion order is `[x, y, z, w]`. Workspace numbers in YAML are examples; retune before hardware.

## Data flow

```text
PICO / mock sender
  → UDP/TCP pico_teleop_v1
  → Unity XR → teleop_standard (+X forward, +Y left, +Z up)
  → yaw-only head-base (hand relative to operator heading)
  → optional filter on hand_rel
  → index-trigger deadman + delayed VR baseline + Δp/Δq limits
  → teleop_to_command (HAL axis_map) + velocity / workspace
  → absolute Cartesian RobotCommand
  → DummyRobotAdapter (default) or Robot_control_interface / airbot_sdk
  → /arm/state  (joint + EE, for Data_Collection)
```

Control method (same as the Quest teleop stack): freeze the robot TCP when the index trigger is pressed; each later frame adds the head-base hand delta onto that TCP. Release the trigger to hold. Side grip is the gripper (`≥0.85` = fully closed). Left primary pauses. Right A (`right_primary`) goes to the configured `go_home_position` (currently joint zeros). Home, pause, and link-timeout require a trigger release before teleop can be re-enabled.

## Layout

```text
app.py
config/default.yaml
protocol/          PicoTeleopPacket, RobotCommand
receiver/          UDP + TCP + RX stats
processing/        vr_normalize, head_base, clutch, transform, filter, limits
robot/             BaseRobotAdapter, DummyRobotAdapter
robot/adapters/    future real SDKs only
tools/             mock_pico_sender, packet_monitor, pose_table
tests/
```

## Install

This Ubuntu image has no `python3.12-venv` apt package (`ensurepip` missing), so a normal `python3 -m venv .venv` will fail. Use:

```bash
cd /home/dt/ego_ARX/EE_controlhal/middle_interface
deactivate 2>/dev/null || true
rm -rf .venv
/usr/bin/python3 -m venv --without-pip --system-site-packages .venv
source .venv/bin/activate
```

`numpy` / `PyYAML` / `pytest` are already on the system Python and satisfy `requirements.txt`. Do **not** run `pip install` against the system interpreter (PEP 668).

If you later need extra packages inside the venv:

```bash
curl -sS https://bootstrap.pypa.io/get-pip.py | python3
python3 -m pip install -r requirements.txt
```

Run every command below from this directory, with the venv active.

## Mock PICO

```bash
python3 -m tools.mock_pico_sender --host 127.0.0.1 --port 15000 --rate 60
```

Default trigger is 0.8 so deadman engages. `--trigger-cycle 3` toggles press/release. `--grip` is the gripper.

## Command monitor

Shows the Cartesian target after head-base / deadman / limits — the signal sent to the robot adapter, not raw headset poses. Occupies port 15000; do not run `app.py` at the same time.

```bash
python3 -m tools.pose_table --hz 60
```

`EE_base` is the robot TCP frozen on trigger press. `d_vr` is the head-base increment. `EE_cmd` is the absolute target being sent. `HOLD` shows `EE_hold` (last command, not updating).

Line mode (0.5 s):

```bash
python3 -m tools.packet_monitor
```

## Middle interface

```bash
python3 app.py --config config/default.yaml
```

Startup prints `ROBOT ADAPTER: DUMMY`. Right index trigger = deadman, right grip = gripper, left primary = pause, right A = go home.

The real `config/airbot.yaml` enables synchronized pose recording. Each received
packet is written as one JSON line under `logs/vr_arm_*.jsonl`, containing both
controller/HMD poses, the actual robot TCP pose before the command, the command
target TCP pose, and teleoperation state. Stop the process with `Ctrl-C` so the
file is closed cleanly.

## Tests

```bash
python3 -m pytest -q
```

## Config

`config/default.yaml`: listen IP/port/timeout, deadman/gripper mapping, head-base, delayed VR baseline, delta limits, `vr_space` (`unity` for this PICO app), `teleop_to_command` axis_map, filter, optional velocity limits, workspace, robot adapter.

Keep `robot.adapter: dummy` until you switch to the command layer:

```bash
python3 app.py --config config/arx.yaml
```

That loads `../Robot_control_interface` (`arx_ros2` backend → `/arm_cmd` mode=4). Source ROS 2 and start `bringup_single_arm.sh` first. Change `transform.axis_map` when the real base axes are not teleop_standard; do not change the head-base / delta math.
