#!/usr/bin/env python3
"""
Scissor action server with closed-loop position verification and retry.

Uses the FollowJointTrajectory action client (not topic publish) to send
goals to the dynamixel controller, then verifies actual position arrival
via joint_states feedback. Retries on failure.
"""

import numpy as np
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from dynamixel_scissors.msg import (
    ScissorControlAction, ScissorControlResult, ScissorControlFeedback)
from scissor_config import ScissorConfig


class ScissorActionServer:

    def __init__(self, config_file=None):
        rospy.init_node('scissor_action_server', anonymous=True)

        self.config = ScissorConfig(config_file)
        self.config.print_config_summary()

        # State
        self.current_position = 0.0
        self.center_position = 0.0
        self.current_effort = 0.0
        self.joint_state_received = False
        self.safety_active = False
        self.last_safety_time = rospy.Time(0)

        # Parameters
        self.joint_name = self.config.get_joint_name()
        self.max_position = self.config.get_max_position()
        self.min_position = self.config.get_min_position()
        self.max_effort = self.config.get_max_effort()
        self.safety_open_distance = self.config.get_safety_open_distance()
        self.safety_cooldown = self.config.get_safety_cooldown()

        self.position_tolerance = rospy.get_param('~position_tolerance', 0.05)
        self.max_retries = rospy.get_param('~max_retries', 3)
        self.settle_time = rospy.get_param('~settle_time', 0.3)
        self.feedback_timeout = float(rospy.get_param(
            '~feedback_timeout', self.config.get_feedback_timeout()))
        self.last_feedback_time = rospy.Time(0)

        # FollowJointTrajectory action client
        fjt_topic = '/ros_scissor/position_joint_trajectory_controller/follow_joint_trajectory'
        self.trajectory_client = actionlib.SimpleActionClient(
            fjt_topic, FollowJointTrajectoryAction)
        rospy.loginfo("[ScissorAction] Waiting for trajectory action server %s ...", fjt_topic)
        if not self.trajectory_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("[ScissorAction] Trajectory action server not available, "
                         "will retry on each goal")

        # Joint state subscriber
        self.joint_sub = rospy.Subscriber(
            self.config.get_joint_states_topic(), JointState,
            self._joint_state_cb, queue_size=1)

        # Wait for initial joint state
        t0 = rospy.Time.now()
        while not self.joint_state_received and (rospy.Time.now() - t0).to_sec() < 5.0:
            rospy.sleep(0.1)
        if self.joint_state_received:
            rospy.loginfo("[ScissorAction] Initial position: %.4f rad", self.current_position)
        else:
            rospy.logwarn("[ScissorAction] No joint state received, using 0.0")

        # ScissorControl action server
        self.server = actionlib.SimpleActionServer(
            'scissor_control', ScissorControlAction,
            self._execute_cb, False)
        self.server.start()
        rospy.loginfo("[ScissorAction] Ready")

    # ------------------------------------------------------------------ #
    #  Joint state
    # ------------------------------------------------------------------ #
    def _joint_state_cb(self, msg):
        try:
            idx = list(msg.name).index(self.joint_name)
            position = msg.position[idx]
        except (ValueError, IndexError):
            return

        effort = msg.effort[idx] if len(msg.effort) > idx else 0.0
        valid, reason = self.config.validate_feedback(position, effort)
        if not valid:
            rospy.logwarn_throttle(
                1.0, '[ScissorAction] rejected invalid feedback: %s', reason)
            return

        self.current_position = float(position)
        self.current_effort = abs(float(effort))
        self.last_feedback_time = rospy.Time.now()
        if not self.joint_state_received:
            self.center_position = self.current_position
            self.joint_state_received = True

        self._check_safety()

    def _feedback_ready(self):
        return bool(
            self.joint_state_received
            and (rospy.Time.now() - self.last_feedback_time).to_sec()
            <= self.feedback_timeout)

    def _check_safety(self):
        if self.current_effort > self.max_effort:
            now = rospy.Time.now()
            if (now - self.last_safety_time).to_sec() > self.safety_cooldown:
                rospy.logwarn("[ScissorAction] SAFETY: effort=%.3f > %.3f, opening",
                             self.current_effort, self.max_effort)
                target = self.config.get_open_step(
                    self.current_position, self.safety_open_distance)
                self._send_trajectory(target, duration=0.5, wait=False)
                self.last_safety_time = now
                self.safety_active = True
        elif self.safety_active and self.current_effort < self.max_effort * 0.8:
            self.safety_active = False

    # ------------------------------------------------------------------ #
    #  Low-level trajectory
    # ------------------------------------------------------------------ #
    def _send_trajectory(self, position, duration=1.0, wait=True):
        """Send a FollowJointTrajectory goal. Returns True if accepted."""
        position = float(np.clip(position, self.min_position, self.max_position))

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]

        self.trajectory_client.send_goal(goal)
        if wait:
            return self.trajectory_client.wait_for_result(
                rospy.Duration(duration + 2.0))
        return True

    def _position_reached(self, target):
        return abs(self.current_position - target) <= self.position_tolerance

    def _move_with_retry(self, target, duration, feedback_cb=None):
        """Send trajectory, verify position, retry on failure.
        Returns (success, message)."""
        target = float(np.clip(target, self.min_position, self.max_position))

        for attempt in range(1, self.max_retries + 1):
            if self.server.is_preempt_requested():
                return False, "Preempted"
            if not self._feedback_ready():
                return False, "Fresh joint feedback unavailable"

            rospy.loginfo("[ScissorAction] Attempt %d/%d: %.3f -> %.3f",
                          attempt, self.max_retries, self.current_position, target)

            self._send_trajectory(target, duration, wait=True)

            # Settle
            rospy.sleep(self.settle_time)

            if not self._feedback_ready():
                return False, "Joint feedback became stale during motion"

            if feedback_cb:
                feedback_cb()

            if self._position_reached(target):
                return True, "Position reached (attempt %d)" % attempt

            rospy.logwarn("[ScissorAction] Position not reached: "
                         "current=%.3f target=%.3f tol=%.3f",
                         self.current_position, target, self.position_tolerance)

        return False, ("Failed after %d retries (current=%.3f, target=%.3f)"
                       % (self.max_retries, self.current_position, target))

    # ------------------------------------------------------------------ #
    #  Action callback
    # ------------------------------------------------------------------ #
    def _execute_cb(self, goal):
        feedback = ScissorControlFeedback()
        result = ScissorControlResult()

        duration = goal.duration if goal.duration > 0 else 1.0

        if not self._feedback_ready():
            result.success = False
            result.message = "Fresh joint feedback unavailable"
            result.final_position = self.current_position
            self.server.set_aborted(result)
            return

        def publish_feedback():
            feedback.current_position = self.current_position
            feedback.current_effort = self.current_effort
            feedback.safety_active = self.safety_active
            feedback.status_message = "Moving"
            self.server.publish_feedback(feedback)

        # Resolve target position
        cmd = goal.command.strip().lower()
        target = None

        if cmd == 'open':
            target = self.config.get_open_step(self.current_position)
        elif cmd == 'close':
            target = self.config.get_close_step(self.current_position)
        elif cmd == 'full_open':
            target = self.config.get_open_position()
        elif cmd == 'full_close':
            target = self.config.get_close_position()
        elif cmd == 'set_position':
            target = goal.position
        elif cmd == 'reset_center':
            target = self.center_position
        elif cmd == 'toggle':
            open_pos = self.config.get_open_position()
            close_pos = self.config.get_close_position()
            if abs(self.current_position - open_pos) > abs(self.current_position - close_pos):
                target = open_pos
            else:
                target = close_pos
        elif cmd == 'set_center':
            self.center_position = self.current_position
            result.success = True
            result.message = "Center set to %.3f" % self.center_position
            result.final_position = self.current_position
            self.server.set_succeeded(result)
            return
        else:
            result.success = False
            result.message = "Unknown command: %s" % cmd
            result.final_position = self.current_position
            self.server.set_aborted(result)
            return

        if target is None:
            result.success = False
            result.message = "Could not resolve target position"
            result.final_position = self.current_position
            self.server.set_aborted(result)
            return

        # Validate
        if target < self.min_position or target > self.max_position:
            result.success = False
            result.message = ("Target %.3f outside limits [%.2f, %.2f]"
                              % (target, self.min_position, self.max_position))
            result.final_position = self.current_position
            self.server.set_aborted(result)
            return

        rospy.loginfo("[ScissorAction] cmd=%s target=%.3f duration=%.1f",
                      cmd, target, duration)

        success, message = self._move_with_retry(target, duration, publish_feedback)

        result.success = success
        result.message = message
        result.final_position = self.current_position

        if success:
            rospy.loginfo("[ScissorAction] %s", message)
            self.server.set_succeeded(result)
        else:
            rospy.logwarn("[ScissorAction] %s", message)
            self.server.set_aborted(result)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', '-c', type=str, default=None)
    args, _ = parser.parse_known_args()

    try:
        ScissorActionServer(config_file=args.config)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
