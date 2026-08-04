# Dynamixel Scissors

ROS 1 driver and action interface for the XC330 four-bar scissors.

## Robot bringup

```bash
roslaunch dynamixel_scissors scissors_on_robot.launch \
  port_name:=/dev/hironx_dxl_tool_bus \
  baud_rate:=57600 \
  publish_legacy_model:=false \
  launch_rviz:=false
```

The launch starts:

- `dynamixel_general_hw` in `/ros_scissor`
- `/ros_scissor/joint_states`
- the position trajectory controller
- `/scissor_control`, a closed-loop `ScissorControlAction` server
- motor-to-blade visualization conversion
- optional connection monitoring

The keyboard node is disabled by default because it publishes directly to the
same trajectory controller. Enable it only for manual commissioning:

```bash
roslaunch dynamixel_scissors scissors_on_robot.launch \
  launch_keyboard_control:=true
```

## Coordinate and motion convention

- motor `+0.50 rad`: closed
- decreasing motor angle: opening
- normal full-open target: `-1.50 rad`
- configured hardware limit: `-3.14 .. +0.50 rad`
- converted blade angle `0 rad`: closed

The wider `-3.14 rad` hardware limit is retained for commissioning, but normal
`full_open` and the HIRONX cut primitive stop at the CAD-validated `-1.50 rad`.

## Safety

Commands are blocked unless fresh joint feedback exists. Samples outside the
configured position envelope, non-finite samples, and implausible effort from
failed sync reads are rejected before they can trigger or satisfy a motion.
Incremental and emergency-open commands move toward the configured open
endpoint, independent of motor sign.

The connection monitor can be disabled while diagnosing a bus problem:

```bash
roslaunch dynamixel_scissors scissors_on_robot.launch \
  enable_connection_monitor:=false
```

Do not run Dynamixel Wizard or another driver on the same serial adapter while
this launch is active.
