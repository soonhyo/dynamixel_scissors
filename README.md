# Scissor Control System

A ROS-based scissor control system with safety features and multiple control options.

## Files

### Single Node (Original)
- **scissor_control_node.py**: All-in-one keyboard control with safety features

### Server-Client Architecture  
- **scissor_server.py**: Core control server with topic interface
- **scissor_client.py**: Simple keyboard client

## Usage

### Single Node
```bash
./scissor_control_node.py
```

### Server-Client Mode
```bash
# Terminal 1: Start server
./scissor_server.py

# Terminal 2: Start keyboard client
./scissor_client.py
```

## Controls

### Basic Movement
- **w/s**: Open/Close scissor (incremental)
- **a/d**: Decrease/Increase increment size
- **r**: Reset to center position
- **c**: Set current position as new center

### Full Range Controls
- **o**: Full open (maximum position)
- **p**: Full close (minimum position)  
- **t**: Toggle scissor (open↔close)

### Safety Controls
- **u/j**: Increase/Decrease max effort threshold
- **k/m**: Increase/Decrease safety open distance

### Information
- **i**: Show current status info
- **q**: Quit

## Safety Features

### Automatic Protection
- **Effort Monitoring**: Continuously monitors joint effort
- **Auto-Open**: Automatically opens when effort exceeds threshold
- **Trajectory Cancellation**: Cancels current movement on safety trigger
- **Cooldown Protection**: 2-second cooldown prevents repeated triggers
- **Auto-Reset**: Safety mode clears when effort returns to normal

### Safety Parameters
- **Max Effort Threshold**: 0.5 (adjustable 0.1-2.0)
- **Safety Open Distance**: 0.2 rad (adjustable 0.05-1.0) 
- **Safety Cooldown**: 2.0 seconds

## Topics

### Control Topics (Server-Client Mode)
```bash
# Command topic (String messages)
/scissor/command

# Example usage
rostopic pub /scissor/command std_msgs/String "data: 'w'"
```

### Robot Interface
```bash
# Publishes to:
/sample_robot/position_joint_trajectory_controller/follow_joint_trajectory/goal

# Subscribes to:
/sample_robot/joint_states
```

## Configuration

### Position Limits
- **Max Position**: 0.50 radians
- **Min Position**: -3.14 radians
- **Default Increment**: 0.05 radians

### Trajectory Settings
- **Movement Duration**: 0.3 seconds
- **Rate Limiting**: 50ms between commands
- **Smooth Trajectories**: Multi-point interpolation

## Status Information

Press **i** to see current status:
- Current position and center position
- Position increment and limits
- Current effort and safety threshold
- Safety parameters and status
- Joint state connection status

## Safety Operation

1. **Normal Operation**: Green - Effort below threshold
2. **Safety Triggered**: Red - High effort detected, auto-opening
3. **Recovery**: System automatically returns to normal when effort decreases

## Installation

### Dependencies

First, install the required dependencies using wstool:

```bash
# From your workspace src directory
wstool init .
wstool merge dynamixel_scissors/.rosinstall
wstool update
```

### ROS Dependencies
- rospy
- control_msgs
- trajectory_msgs  
- actionlib_msgs
- sensor_msgs
- std_msgs
- dynamixel_general_hw

## Example Usage

```bash
# Basic control
./scissor_control_node.py

# Adjust safety threshold  
# Press 'u' multiple times to increase effort threshold
# Press 'k' to increase safety opening distance

# Monitor status
# Press 'i' to see current effort and safety parameters

# Emergency: If scissors get stuck, they will automatically open (friction is not applied)
```
