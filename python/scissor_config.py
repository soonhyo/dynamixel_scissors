#!/usr/bin/env python3

import yaml
import rospy
import os
from typing import Dict, Any, Optional

class ScissorConfig:
    """Configuration manager for the scissor control system"""

    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize configuration from YAML file

        Args:
            config_file: Path to YAML configuration file. If None, uses default.
        """
        self.config_data = {}
        self.config_file = config_file

        if config_file is None:
            # Use default config file
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.config_file = os.path.join(package_path, 'config', 'scissor_config.yaml')

        self.load_config()
        self._validate_config()

    def load_config(self):
        """Load configuration from YAML file"""
        try:
            with open(self.config_file, 'r') as file:
                self.config_data = yaml.safe_load(file)
                rospy.loginfo(f"Loaded configuration from: {self.config_file}")
        except FileNotFoundError:
            rospy.logerr(f"Configuration file not found: {self.config_file}")
            rospy.loginfo("Using default configuration")
            self._load_default_config()
        except yaml.YAMLError as e:
            rospy.logerr(f"Error parsing YAML file: {e}")
            rospy.loginfo("Using default configuration")
            self._load_default_config()
        except Exception as e:
            rospy.logerr(f"Unexpected error loading config: {e}")
            rospy.loginfo("Using default configuration")
            self._load_default_config()

    def _load_default_config(self):
        """Load default configuration when file is not available"""
        self.config_data = {
            'hardware': {
                'joint_name': 'sample_joint',
                'position_limits': {
                    'min_position': -3.14,
                    'max_position': 0.50
                },
                'position_mapping': {
                    'open_is_max': True,
                    'custom_open_position': None,
                    'custom_close_position': None
                }
            },
            'control': {
                'position_increment': 0.05,
                'min_command_interval': 0.05,
                'trajectory': {
                    'default_duration': 0.3,
                    'full_motion_duration': 1.0,
                    'emergency_duration': 0.5
                }
            },
            'safety': {
                'max_effort': 1.0,
                'safety_open_distance': 0.2,
                'safety_cooldown': 2.0,
                'effort_normal_threshold': 0.8
            },
            'topics': {
                'trajectory_goal': '/sample_robot/position_joint_trajectory_controller/follow_joint_trajectory/goal',
                'joint_states': '/sample_robot/joint_states'
            },
            'display': {
                'show_position_precision': 3,
                'show_effort_precision': 3
            }
        }

    def _validate_config(self):
        """Validate configuration values"""
        try:
            # Validate position limits
            min_pos = self.get_min_position()
            max_pos = self.get_max_position()
            if min_pos >= max_pos:
                raise ValueError(f"min_position ({min_pos}) must be less than max_position ({max_pos})")

            # Validate custom positions if specified
            custom_open = self.get_custom_open_position()
            custom_close = self.get_custom_close_position()

            if custom_open is not None:
                if not (min_pos <= custom_open <= max_pos):
                    raise ValueError(f"custom_open_position ({custom_open}) must be within limits [{min_pos}, {max_pos}]")

            if custom_close is not None:
                if not (min_pos <= custom_close <= max_pos):
                    raise ValueError(f"custom_close_position ({custom_close}) must be within limits [{min_pos}, {max_pos}]")

            # Validate safety parameters
            if self.get_max_effort() <= 0:
                raise ValueError("max_effort must be positive")

            if self.get_safety_cooldown() < 0:
                raise ValueError("safety_cooldown must be non-negative")

            rospy.loginfo("Configuration validation passed")

        except Exception as e:
            rospy.logerr(f"Configuration validation failed: {e}")
            raise

    def get(self, key_path: str, default=None):
        """
        Get configuration value using dot notation

        Args:
            key_path: Dot-separated path to the configuration value
            default: Default value if key is not found

        Returns:
            Configuration value or default
        """
        keys = key_path.split('.')
        value = self.config_data

        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default

    # Hardware configuration getters
    def get_joint_name(self) -> str:
        return self.get('hardware.joint_name', 'sample_joint')

    def get_min_position(self) -> float:
        return self.get('hardware.position_limits.min_position', -3.14)

    def get_max_position(self) -> float:
        return self.get('hardware.position_limits.max_position', 0.50)

    def is_open_max(self) -> bool:
        return self.get('hardware.position_mapping.open_is_max', True)

    def get_custom_open_position(self) -> Optional[float]:
        return self.get('hardware.position_mapping.custom_open_position')

    def get_custom_close_position(self) -> Optional[float]:
        return self.get('hardware.position_mapping.custom_close_position')

    def get_open_position(self) -> float:
        """Get the actual open position based on configuration"""
        custom_open = self.get_custom_open_position()
        if custom_open is not None:
            return custom_open

        if self.is_open_max():
            return self.get_max_position()
        else:
            return self.get_min_position()

    def get_close_position(self) -> float:
        """Get the actual close position based on configuration"""
        custom_close = self.get_custom_close_position()
        if custom_close is not None:
            return custom_close

        if self.is_open_max():
            return self.get_min_position()
        else:
            return self.get_max_position()

    # Control configuration getters
    def get_position_increment(self) -> float:
        return self.get('control.position_increment', 0.05)

    def get_min_command_interval(self) -> float:
        return self.get('control.min_command_interval', 0.05)

    def get_default_duration(self) -> float:
        return self.get('control.trajectory.default_duration', 0.3)

    def get_full_motion_duration(self) -> float:
        return self.get('control.trajectory.full_motion_duration', 1.0)

    def get_emergency_duration(self) -> float:
        return self.get('control.trajectory.emergency_duration', 0.5)

    # Safety configuration getters
    def get_max_effort(self) -> float:
        return self.get('safety.max_effort', 1.0)

    def get_safety_open_distance(self) -> float:
        return self.get('safety.safety_open_distance', 0.2)

    def get_safety_cooldown(self) -> float:
        return self.get('safety.safety_cooldown', 2.0)

    def get_effort_normal_threshold(self) -> float:
        return self.get('safety.effort_normal_threshold', 0.8)

    # Topic configuration getters
    def get_trajectory_goal_topic(self) -> str:
        return self.get('topics.trajectory_goal',
                       '/sample_robot/position_joint_trajectory_controller/follow_joint_trajectory/goal')

    def get_joint_states_topic(self) -> str:
        return self.get('topics.joint_states', '/sample_robot/joint_states')

    # Display configuration getters
    def get_position_precision(self) -> int:
        return self.get('display.show_position_precision', 3)

    def get_effort_precision(self) -> int:
        return self.get('display.show_effort_precision', 3)

    def print_config_summary(self):
        """Print a summary of the current configuration"""
        rospy.loginfo("=== Scissor Configuration Summary ===")
        rospy.loginfo(f"Config file: {self.config_file}")
        rospy.loginfo(f"Joint name: {self.get_joint_name()}")
        rospy.loginfo(f"Position limits: [{self.get_min_position():.2f}, {self.get_max_position():.2f}] rad")
        rospy.loginfo(f"Open is max: {self.is_open_max()}")
        rospy.loginfo(f"Open position: {self.get_open_position():.3f} rad")
        rospy.loginfo(f"Close position: {self.get_close_position():.3f} rad")
        rospy.loginfo(f"Max effort: {self.get_max_effort():.2f}")
        rospy.loginfo(f"Safety open distance: {self.get_safety_open_distance():.2f}")
        rospy.loginfo("====================================")