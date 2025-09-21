# Scissor Control Configuration

This directory contains configuration files for the scissor control system, which allows you to configure open/close positions and hardware-specific settings.

## Configuration Structure

The main configuration file is `scissor_config.yaml`, which contains all configurable parameters:

- **Hardware Configuration**: Joint name, position limits, and open/close mapping
- **Control Parameters**: Movement increments, trajectory durations
- **Safety Configuration**: Effort limits and safety behaviors
- **ROS Topic Configuration**: Topic names for communication
- **Display Configuration**: Precision for displayed values

## Hardware Position Mapping

The key feature is the `position_mapping` section that handles different hardware orientations:

### Standard Hardware (open_is_max: true)
- `max_position` corresponds to **open** state
- `min_position` corresponds to **close** state

### Inverted Hardware (open_is_max: false)
- `max_position` corresponds to **close** state
- `min_position` corresponds to **open** state

### Custom Positions
You can specify exact positions for open/close states:
```yaml
position_mapping:
  open_is_max: true  # This is ignored when custom positions are set
  custom_open_position: 0.3   # Specific open position
  custom_close_position: -2.0 # Specific close position
```

## Example Configurations

### hardware_examples/standard_scissor.yaml
Standard configuration where max_position = open state

### hardware_examples/inverted_scissor.yaml
Inverted configuration where max_position = close state

### hardware_examples/custom_positions.yaml
Custom configuration with specific open/close positions

## Usage

### 1. Using Default Configuration
```bash
python3 scissor_control_node.py
```

### 2. Using Specific Configuration File
```bash
python3 scissor_control_node.py --config /path/to/config.yaml
```

### 3. Using Example Configurations
```bash
# For standard hardware
python3 scissor_control_node.py --config config/hardware_examples/standard_scissor.yaml

# For inverted hardware
python3 scissor_control_node.py --config config/hardware_examples/inverted_scissor.yaml

# For custom positions
python3 scissor_control_node.py --config config/hardware_examples/custom_positions.yaml
```

## Configuration Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `hardware.joint_name` | ROS joint name | "sample_joint" |
| `hardware.position_limits.min_position` | Minimum joint position (rad) | -3.14 |
| `hardware.position_limits.max_position` | Maximum joint position (rad) | 0.50 |
| `hardware.position_mapping.open_is_max` | If true, max_position=open | true |
| `hardware.position_mapping.custom_open_position` | Custom open position | null |
| `hardware.position_mapping.custom_close_position` | Custom close position | null |
| `control.position_increment` | Manual control increment | 0.05 |
| `control.trajectory.default_duration` | Default movement time (s) | 0.3 |
| `control.trajectory.full_motion_duration` | Full open/close time (s) | 1.0 |
| `safety.max_effort` | Maximum effort threshold | 1.0 |
| `safety.safety_open_distance` | Emergency open distance | 0.2 |

## Testing Configuration

Run the configuration test script to verify your setup:
```bash
python3 test_config.py
```

This will test:
- Default configuration loading
- Example configuration files
- Position mapping logic
- Error handling for invalid files

## Creating Custom Configurations

1. Copy one of the example files:
   ```bash
   cp hardware_examples/standard_scissor.yaml my_scissor_config.yaml
   ```

2. Edit the configuration:
   - Set `joint_name` to match your robot's joint
   - Adjust `position_limits` for your hardware range
   - Configure `position_mapping` based on your hardware orientation
   - Tune `safety` parameters for your specific setup

3. Test your configuration:
   ```bash
   python3 scissor_control_node.py --config my_scissor_config.yaml
   ```

## Troubleshooting

**Configuration not loading**: Check file path and YAML syntax
**Validation errors**: Ensure position limits are valid and custom positions are within limits
**Position mapping issues**: Verify `open_is_max` setting matches your hardware orientation

For hardware where the mechanical setup is inverted (max joint position actually closes the scissor), set `open_is_max: false` in your configuration.