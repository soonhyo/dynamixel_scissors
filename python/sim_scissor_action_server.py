#!/usr/bin/env python3
"""Simulation-only ScissorControl action server.

It mirrors the hardware server's public action contract while publishing the
commanded source joint on ``/ros_scissor/joint_states``.  The existing
scissors-model joint-state relay therefore animates the articulated CAD in
preview/fake-robot runs.  No hardware controller or feedback is emulated.
"""

import actionlib
import rospy
from sensor_msgs.msg import JointState

from dynamixel_scissors.msg import (
    ScissorControlAction,
    ScissorControlFeedback,
    ScissorControlResult,
)


class SimScissorActionServer:
    def __init__(self):
        self.joint_name = rospy.get_param('~joint_name', 'scissor_joint')
        self.joint_topic = rospy.get_param(
            '~joint_state_topic', '/ros_scissor/joint_states')
        self.minimum = float(rospy.get_param('~minimum_position', -3.14))
        self.maximum = float(rospy.get_param('~maximum_position', 0.50))
        self.open_position = float(rospy.get_param(
            '~open_position', self.minimum))
        self.close_position = float(rospy.get_param(
            '~close_position', self.maximum))
        self.increment = abs(float(rospy.get_param(
            '~position_increment', 0.05)))
        self.rate_hz = max(10.0, float(rospy.get_param('~rate', 60.0)))
        self.position = min(self.maximum, max(
            self.minimum, float(rospy.get_param(
                '~initial_position', self.close_position))))

        self.joint_pub = rospy.Publisher(
            self.joint_topic, JointState, queue_size=1, latch=True)
        self.server = actionlib.SimpleActionServer(
            '/scissor_control', ScissorControlAction,
            execute_cb=self._execute, auto_start=False)
        self.server.start()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._publish_joint)
        rospy.loginfo(
            '[SimScissors] Ready: action=/scissor_control joint=%s topic=%s',
            self.joint_name, self.joint_topic)

    def _joint_message(self):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = [self.joint_name]
        message.position = [self.position]
        message.velocity = [0.0]
        message.effort = [0.0]
        return message

    def _publish_joint(self, _event=None):
        self.joint_pub.publish(self._joint_message())

    def _target(self, goal):
        command = str(goal.command or '').strip().lower()
        if command in ('set_center', 'reset_center'):
            return self.position
        if command == 'set_position':
            return float(goal.position)
        if command == 'full_open':
            return self.open_position
        if command == 'full_close':
            return self.close_position
        if command == 'open':
            return self.position - self.increment
        if command == 'close':
            return self.position + self.increment
        if command == 'toggle':
            midpoint = 0.5 * (self.open_position + self.close_position)
            return (self.open_position if self.position > midpoint
                    else self.close_position)
        raise ValueError('unknown command: {}'.format(command))

    def _result(self, success, message):
        result = ScissorControlResult()
        result.success = bool(success)
        result.message = str(message)
        result.final_position = float(self.position)
        return result

    def _execute(self, goal):
        try:
            target = self._target(goal)
        except ValueError as exc:
            self.server.set_aborted(self._result(False, str(exc)))
            return
        if target < self.minimum or target > self.maximum:
            self.server.set_aborted(self._result(
                False,
                'target {:.3f} outside [{:.3f}, {:.3f}]'.format(
                    target, self.minimum, self.maximum)))
            return

        start = float(self.position)
        duration = max(0.05, float(goal.duration or 0.0))
        started = rospy.Time.now()
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted(self._result(False, 'preempted'))
                return
            elapsed = (rospy.Time.now() - started).to_sec()
            alpha = min(1.0, elapsed / duration)
            # Smoothstep avoids an artificial velocity discontinuity in the
            # articulated scissors preview.
            blend = alpha * alpha * (3.0 - 2.0 * alpha)
            self.position = start + blend * (target - start)
            self._publish_joint()

            feedback = ScissorControlFeedback()
            feedback.current_position = float(self.position)
            feedback.current_effort = 0.0
            feedback.safety_active = False
            feedback.status_message = 'simulating'
            self.server.publish_feedback(feedback)
            if alpha >= 1.0:
                break
            rate.sleep()

        self.position = float(target)
        self._publish_joint()
        self.server.set_succeeded(self._result(True, 'simulated position reached'))


def main():
    rospy.init_node('sim_scissor_action_server')
    SimScissorActionServer()
    rospy.spin()


if __name__ == '__main__':
    main()
