# Revo2 hand feedback collection

This is a standalone collector for the Revo2 physical hand. It records neither
Manus glove data nor retargeting output nor hand control targets. The VIVE
process is the only CM6K HID reader: its right pedal (`KEY_2`) publishes a ROS
recording gate, which this collector receives through rosbridge.

It follows the established ROS 2 and LeRobot pattern:

```text
CM6K right pedal -> VIVE -> /teleop/hand_recording (std_msgs/Bool) -> rosbridge
Revo2 driver -> ros2_control JointStateBroadcaster -> sensor_msgs/JointState
             -> rosbridge -> latest-state callback -> 10 Hz LeRobot add_frame
             -> right-pedal release -> save_episode -> Parquet
```

The source topic is the physical feedback published by `revo2_joint_state`:

```text
/revo2_right/revo2_joint_state/joint_states
```

Do not use `/retarget/joint_states` or
`/revo2_right/revo2_pid_controller/target_joint_states`. They are retargeting or
control-target streams, not the Revo2 hand's measured state.

## Recorded schema

Every row is one 10 Hz observation. LeRobot adds its own `timestamp`,
`frame_index`, and `episode_index` columns.

```text
observation.host_monotonic_ns             int64[1]
observation.state.hand_right_joint        float32[6]  rad
observation.state.hand_right_velocity     float32[6]  rad/s
observation.state.hand_right_valid        float32[1]  0 or 1
```

The fixed six-element order is:

```text
right_thumb_proximal_joint
right_thumb_metacarpal_joint
right_index_proximal_joint
right_middle_proximal_joint
right_ring_proximal_joint
right_pinky_proximal_joint
```

The adapter uses `JointState.name` to apply this order before writing. A stale,
incomplete, or non-finite message becomes zero arrays with `valid=0`, preserving
a fixed Parquet schema. No `action` feature is written because this dataset only
contains measured hand feedback.

`dynamic_joint_states` has diagnostic extensions such as `current` and
`motor_state`. `current` is the SDK raw motor-current reading, not joint torque
or a documented physical unit. `motor_state` is the SDK `uint8` status code:
`0=idle`, `1=running`, `2=stalled`, `3=turbo`. They are deliberately absent
from this motion-feedback dataset.

## Run

First start the Revo2 driver and verify the feedback topic in the ROS 2
environment that owns the hand:

```bash
ros2 topic echo --once /revo2_right/revo2_joint_state/joint_states
```

Start a rosbridge websocket in that same ROS domain. When it runs inside a
container, expose or forward port 9090 to the host before starting this collector.
The collector waits for one complete hand-feedback message for up to 10 seconds;
it does not create an all-zero episode when the topic is absent.

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Install the collection environment once. Start VIVE teleop first; it publishes
`/teleop/hand_recording` at 20 Hz. The collector waits without writing frames
until the right CM6K pedal is held. Each press-to-release interval is one
LeRobot episode. On VIVE/ROS failure the gate becomes stale after 0.5 seconds,
which saves the current episode and stops frame writes.

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/Data_Collection
.venv-collect/bin/pip install -r hand_collect/requirements.txt
.venv-collect/bin/python hand_collect/collect.py
```

`--seconds 30` remains available as a 30-second collector lifetime cap for a
short check. It writes frames only while the right pedal is held.

The dataset is written to
`Data_Collection/data/lerobot/revo2_right_feedback`. To record the left hand,
change only `hand.side`, `hand.topic`, and `dataset.repo_id` in a copied config.

## Files

- `hand_adapter.py`: defines the driver joint order, validates and normalizes
  ROS `JointState`, and declares the fixed LeRobot schema.
- `recorder.py`: subscribes through rosbridge, retains just the newest feedback,
  samples it at 10 Hz, and saves a LeRobot episode.
- `collect.py`: command-line entry point.
- `config/revo2_right.yaml`: the deployed right-hand topic, output ID, and rate.
- `tests/test_hand_adapter.py`: verifies name reordering and invalid-state shape.
