#!/usr/bin/env python3

import os
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'python'))

from scissor_config import ScissorConfig  # noqa: E402


class ScissorConfigTest(unittest.TestCase):

    def setUp(self):
        self.config = ScissorConfig(os.path.join(
            ROOT, 'config', 'scissor_config.yaml'))

    def test_incremental_commands_follow_configured_endpoints(self):
        self.assertAlmostEqual(-0.05, self.config.get_open_step(0.0))
        self.assertAlmostEqual(0.05, self.config.get_close_step(0.0))
        self.assertAlmostEqual(-1.50, self.config.get_open_step(-1.49, 0.05))
        self.assertAlmostEqual(0.50, self.config.get_close_step(0.48, 0.05))

    def test_failed_sync_read_samples_are_rejected(self):
        self.assertEqual((True, ''), self.config.validate_feedback(0.5, 0.1))
        self.assertFalse(self.config.validate_feedback(3003918.0, 90.23)[0])
        self.assertFalse(self.config.validate_feedback(0.0, 90.23)[0])
        self.assertFalse(self.config.validate_feedback(float('nan'), 0.0)[0])


if __name__ == '__main__':
    unittest.main()
