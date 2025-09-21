#!/usr/bin/env python3

import sys
import os
from scissor_config import ScissorConfig

def test_config_loading():
    """Test configuration loading and validation"""
    print("=== Testing Configuration System ===\n")

    # Test 1: Default configuration
    print("1. Testing default configuration:")
    try:
        config = ScissorConfig()
        print("✓ Default configuration loaded successfully")
        config.print_config_summary()
    except Exception as e:
        print(f"✗ Default configuration failed: {e}")
    print()

    # Test 2: Standard scissor configuration
    print("2. Testing standard scissor configuration:")
    try:
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_file = os.path.join(package_path, 'config', 'hardware_examples', 'standard_scissor.yaml')
        config = ScissorConfig(config_file)
        print("✓ Standard scissor configuration loaded successfully")
        print(f"   Open position: {config.get_open_position():.3f} rad")
        print(f"   Close position: {config.get_close_position():.3f} rad")
        print(f"   Open is max: {config.is_open_max()}")
    except Exception as e:
        print(f"✗ Standard scissor configuration failed: {e}")
    print()

    # Test 3: Inverted scissor configuration
    print("3. Testing inverted scissor configuration:")
    try:
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_file = os.path.join(package_path, 'config', 'hardware_examples', 'inverted_scissor.yaml')
        config = ScissorConfig(config_file)
        print("✓ Inverted scissor configuration loaded successfully")
        print(f"   Open position: {config.get_open_position():.3f} rad")
        print(f"   Close position: {config.get_close_position():.3f} rad")
        print(f"   Open is max: {config.is_open_max()}")
    except Exception as e:
        print(f"✗ Inverted scissor configuration failed: {e}")
    print()

    # Test 4: Custom positions configuration
    print("4. Testing custom positions configuration:")
    try:
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_file = os.path.join(package_path, 'config', 'hardware_examples', 'custom_positions.yaml')
        config = ScissorConfig(config_file)
        print("✓ Custom positions configuration loaded successfully")
        print(f"   Open position: {config.get_open_position():.3f} rad")
        print(f"   Close position: {config.get_close_position():.3f} rad")
        print(f"   Custom open: {config.get_custom_open_position()}")
        print(f"   Custom close: {config.get_custom_close_position()}")
    except Exception as e:
        print(f"✗ Custom positions configuration failed: {e}")
    print()

    # Test 5: Non-existent file (should fall back to defaults)
    print("5. Testing non-existent configuration file:")
    try:
        config = ScissorConfig("/non/existent/file.yaml")
        print("✓ Non-existent file handled gracefully (using defaults)")
        print(f"   Open position: {config.get_open_position():.3f} rad")
        print(f"   Close position: {config.get_close_position():.3f} rad")
    except Exception as e:
        print(f"✗ Non-existent file handling failed: {e}")
    print()

def test_position_mapping():
    """Test position mapping logic"""
    print("=== Testing Position Mapping Logic ===\n")

    # Create test configurations
    test_configs = [
        {
            'name': 'Standard (open_is_max=True)',
            'config': {
                'hardware': {
                    'position_limits': {'min_position': -3.14, 'max_position': 0.50},
                    'position_mapping': {'open_is_max': True, 'custom_open_position': None, 'custom_close_position': None}
                }
            },
            'expected_open': 0.50,
            'expected_close': -3.14
        },
        {
            'name': 'Inverted (open_is_max=False)',
            'config': {
                'hardware': {
                    'position_limits': {'min_position': -3.14, 'max_position': 0.50},
                    'position_mapping': {'open_is_max': False, 'custom_open_position': None, 'custom_close_position': None}
                }
            },
            'expected_open': -3.14,
            'expected_close': 0.50
        },
        {
            'name': 'Custom positions',
            'config': {
                'hardware': {
                    'position_limits': {'min_position': -3.14, 'max_position': 0.50},
                    'position_mapping': {'open_is_max': True, 'custom_open_position': 0.3, 'custom_close_position': -2.0}
                }
            },
            'expected_open': 0.3,
            'expected_close': -2.0
        }
    ]

    for test in test_configs:
        print(f"Testing {test['name']}:")
        try:
            # Create a temporary config object by directly setting config_data
            config = ScissorConfig()
            config.config_data = test['config']

            open_pos = config.get_open_position()
            close_pos = config.get_close_position()

            if abs(open_pos - test['expected_open']) < 0.001 and abs(close_pos - test['expected_close']) < 0.001:
                print(f"✓ Correct positions - Open: {open_pos:.3f}, Close: {close_pos:.3f}")
            else:
                print(f"✗ Incorrect positions - Expected Open: {test['expected_open']:.3f}, Close: {test['expected_close']:.3f}")
                print(f"  Got Open: {open_pos:.3f}, Close: {close_pos:.3f}")
        except Exception as e:
            print(f"✗ Test failed: {e}")
        print()

if __name__ == '__main__':
    test_config_loading()
    test_position_mapping()
    print("=== Configuration Testing Complete ===")