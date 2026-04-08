#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Converts Dynamixel motor angle to actual scissors blade angle
using 4-bar linkage kinematics, and publishes for RViz visualization.

Subscribe: /ros_scissor/joint_states (motor angle)
Publish:   /scissors/joint_states    (converted scissors blade angle)
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState


class ScissorsAngleConverter:
    def __init__(self):
        rospy.init_node('scissors_angle_converter', anonymous=True)

        # 4-bar linkage parameters (meters)
        # O1: motor shaft, O2: scissors pivot (origin)
        motor_x = rospy.get_param('~motor_x', 0.01)     # 1cm
        motor_z = rospy.get_param('~motor_z', -0.070)    # -7cm
        self.O1 = np.array([motor_x, motor_z])
        self.O2 = np.array([0.0, 0.0])

        self.a2 = rospy.get_param('~crank_length', 0.022)    # 2.2cm
        self.a3 = rospy.get_param('~coupler_length', 0.067)  # 6.7cm
        self.a4 = rospy.get_param('~rocker_length', 0.067)   # 6.7cm

        # Assembly mode: 0 or 1
        self.assembly_mode = rospy.get_param('~assembly_mode', 0)

        # Joint name in the URDF
        self.joint_name = rospy.get_param('~joint_name', 'scissor_joint')

        # Source joint name from hardware
        self.source_joint_name = rospy.get_param('~source_joint_name', 'scissor_joint')

        # Reference angle offset: scissors angle when blades are closed (aligned)
        # Used to convert linkage output to URDF joint angle
        self.closed_motor_angle = rospy.get_param('~closed_motor_angle', 0.50)
        result = self._compute_linkage(self.closed_motor_angle)
        if result is not None:
            self.closed_scissors_angle = result
            rospy.loginfo("Closed position: motor=%.3f rad -> linkage=%.1f deg",
                          self.closed_motor_angle, np.degrees(self.closed_scissors_angle))
        else:
            self.closed_scissors_angle = 0.0
            rospy.logwarn("Could not compute closed position angle")

        # Publisher (converted angles for robot_state_publisher -> RViz)
        self.pub = rospy.Publisher(
            '/ros_scissor/joint_states_converted', JointState, queue_size=10)

        # Subscriber (raw motor angles from hardware)
        self.sub = rospy.Subscriber(
            '/ros_scissor/joint_states', JointState,
            self.joint_state_callback, queue_size=1)

        rospy.loginfo("ScissorsAngleConverter started")
        rospy.loginfo("  Linkage: crank=%.1fmm, coupler=%.1fmm, rocker=%.1fmm",
                      self.a2 * 1000, self.a3 * 1000, self.a4 * 1000)

    def _compute_linkage(self, motor_angle):
        """Compute scissors angle from motor angle via 4-bar linkage."""
        A = self.O1 + self.a2 * np.array([np.cos(motor_angle),
                                           np.sin(motor_angle)])
        d = np.linalg.norm(A - self.O2)

        if d > self.a3 + self.a4 or d < abs(self.a3 - self.a4):
            return None

        cos_alpha = (self.a4**2 + d**2 - self.a3**2) / (2.0 * self.a4 * d)
        cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
        alpha = np.arccos(cos_alpha)

        phi = np.arctan2(A[1] - self.O2[1], A[0] - self.O2[0])

        if self.assembly_mode == 0:
            return phi + alpha
        else:
            return phi - alpha

    def joint_state_callback(self, msg):
        """Receive motor joint state, convert, and republish."""
        try:
            idx = list(msg.name).index(self.source_joint_name)
        except ValueError:
            return

        motor_angle = msg.position[idx]
        scissors_angle = self._compute_linkage(motor_angle)

        if scissors_angle is None:
            rospy.logwarn_throttle(1.0, "No linkage solution for motor=%.3f", motor_angle)
            return

        # Convert: subtract closed reference so closed=0
        blade_angle = scissors_angle - self.closed_scissors_angle

        # Publish converted joint state
        out = JointState()
        out.header = msg.header
        out.name = [self.joint_name]
        out.position = [blade_angle]
        if len(msg.velocity) > idx:
            out.velocity = [msg.velocity[idx]]
        if len(msg.effort) > idx:
            out.effort = [msg.effort[idx]]

        self.pub.publish(out)


if __name__ == '__main__':
    try:
        node = ScissorsAngleConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
