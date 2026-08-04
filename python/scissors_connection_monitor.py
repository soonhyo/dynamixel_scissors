#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Monitors Dynamixel scissors connection via /ros_scissor/joint_states.
Detects communication failure between DXHub and Dynamixel motor by
checking for out-of-range position/effort values (garbage data from
failed syncRead). On detection, kills the dynamixel_general_control
node (respawn restarts it) and re-loads ros_control controllers.
"""

import subprocess
import rospy
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import (
    ListControllers, LoadController, SwitchController, SwitchControllerRequest)


class ScissorsConnectionMonitor:

    CONTROLLERS = [
        'joint_state_controller',
        'position_joint_trajectory_controller',
    ]

    def __init__(self):
        rospy.init_node('scissors_connection_monitor', anonymous=False)

        # Parameters
        self.check_rate = rospy.get_param('~check_rate', 2.0)
        self.control_node_name = rospy.get_param(
            '~control_node_name', '/ros_scissor/dynamixel_general_control')
        self.joint_name = rospy.get_param('~joint_name', 'scissor_joint')

        # Valid ranges for detecting garbage data
        self.position_min = rospy.get_param('~position_min', -3.2)
        self.position_max = rospy.get_param('~position_max', 0.6)
        self.effort_max = rospy.get_param('~effort_max', 10.0)

        # How many consecutive bad readings before triggering restart
        self.bad_reading_threshold = rospy.get_param('~bad_reading_threshold', 3)
        # Cooldown after restart before monitoring again
        self.restart_cooldown = rospy.get_param('~restart_cooldown', 10.0)
        # The position controller clamps an actuator that powers up outside
        # its URDF limit. Do not mistake that initial convergence for corrupt
        # syncRead data and kill the driver while it is becoming ready.
        self.startup_grace = rospy.get_param('~startup_grace', 2.0)
        self.started_time = rospy.Time.now()

        self.consecutive_bad_readings = 0
        self.is_connected = False
        self.restart_count = 0
        self.last_restart_time = None

        # Controller manager service proxies
        self._cm_ns = '/ros_scissor/controller_manager'
        self.srv_list = None
        self.srv_load = None
        self.srv_switch = None

        self.sub = rospy.Subscriber(
            '/ros_scissor/joint_states', JointState,
            self._joint_state_cb, queue_size=1)

        rospy.loginfo("[ScissorsMonitor] Started -- joint=%s, "
                      "valid position=[%.2f, %.2f], max effort=%.1f, "
                      "bad threshold=%d",
                      self.joint_name,
                      self.position_min, self.position_max,
                      self.effort_max, self.bad_reading_threshold)

    # ------------------------------------------------------------------ #
    #  Callback
    # ------------------------------------------------------------------ #
    def _joint_state_cb(self, msg):
        if ((rospy.Time.now() - self.started_time).to_sec()
                < self.startup_grace):
            return
        # Skip during cooldown
        if (self.last_restart_time is not None and
                (rospy.Time.now() - self.last_restart_time).to_sec() < self.restart_cooldown):
            return

        try:
            idx = list(msg.name).index(self.joint_name)
        except ValueError:
            return

        position = msg.position[idx] if len(msg.position) > idx else 0.0
        effort = abs(msg.effort[idx]) if len(msg.effort) > idx else 0.0

        is_bad = (position < self.position_min or
                  position > self.position_max or
                  effort > self.effort_max)

        if is_bad:
            self.consecutive_bad_readings += 1
            if self.consecutive_bad_readings == self.bad_reading_threshold:
                rospy.logwarn("[ScissorsMonitor] Bad data detected "
                             "(pos=%.2f, effort=%.2f) x%d -- triggering restart",
                             position, effort, self.consecutive_bad_readings)
                self._handle_disconnect()
        else:
            if self.consecutive_bad_readings > 0:
                self.consecutive_bad_readings = 0
            if not self.is_connected:
                self.is_connected = True
                rospy.loginfo("[ScissorsMonitor] Connection OK")

    # ------------------------------------------------------------------ #
    #  Helpers
    # ------------------------------------------------------------------ #
    def _kill_node(self, node_name):
        try:
            subprocess.call(
                ['rosnode', 'kill', node_name],
                timeout=5.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
        except Exception as e:
            rospy.logwarn("[ScissorsMonitor] Failed to kill %s: %s", node_name, e)

    # ------------------------------------------------------------------ #
    #  Controller manager
    # ------------------------------------------------------------------ #
    def _wait_for_controller_manager(self, timeout=30.0):
        svc_list = self._cm_ns + '/list_controllers'
        rospy.loginfo("[ScissorsMonitor] Waiting for controller_manager (%s) ...", svc_list)
        try:
            rospy.wait_for_service(svc_list, timeout=timeout)
        except rospy.ROSException:
            rospy.logerr("[ScissorsMonitor] controller_manager not available after %.0fs", timeout)
            return False

        self.srv_list = rospy.ServiceProxy(
            self._cm_ns + '/list_controllers', ListControllers)
        self.srv_load = rospy.ServiceProxy(
            self._cm_ns + '/load_controller', LoadController)
        self.srv_switch = rospy.ServiceProxy(
            self._cm_ns + '/switch_controller', SwitchController)
        return True

    def _load_and_start_controllers(self):
        try:
            resp = self.srv_list()
            loaded = {c.name for c in resp.controller}
            running = {c.name for c in resp.controller if c.state == 'running'}
        except rospy.ServiceException as e:
            rospy.logerr("[ScissorsMonitor] list_controllers failed: %s", e)
            return False

        to_start = []
        for name in self.CONTROLLERS:
            if name not in loaded:
                rospy.loginfo("[ScissorsMonitor] Loading controller: %s", name)
                try:
                    res = self.srv_load(name)
                    if not res.ok:
                        rospy.logerr("[ScissorsMonitor] Failed to load %s", name)
                        return False
                except rospy.ServiceException as e:
                    rospy.logerr("[ScissorsMonitor] load_controller(%s) failed: %s", name, e)
                    return False
            if name not in running:
                to_start.append(name)

        if to_start:
            rospy.loginfo("[ScissorsMonitor] Starting controllers: %s", to_start)
            try:
                req = SwitchControllerRequest()
                req.start_controllers = to_start
                req.stop_controllers = []
                req.strictness = SwitchControllerRequest.BEST_EFFORT
                res = self.srv_switch(req)
                if not res.ok:
                    rospy.logerr("[ScissorsMonitor] switch_controller failed")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr("[ScissorsMonitor] switch_controller failed: %s", e)
                return False

        rospy.loginfo("[ScissorsMonitor] Controllers ready")
        return True

    # ------------------------------------------------------------------ #
    #  Disconnect handler
    # ------------------------------------------------------------------ #
    def _handle_disconnect(self):
        self.restart_count += 1
        self.is_connected = False
        self.consecutive_bad_readings = 0
        self.last_restart_time = rospy.Time.now()

        rospy.logwarn("[ScissorsMonitor] Restarting dynamixel node (restart #%d) ...",
                      self.restart_count)

        # Kill the control node (respawn will restart it)
        self._kill_node(self.control_node_name)

        # Wait for the respawned node's controller_manager
        if not self._wait_for_controller_manager():
            return

        # Re-load and start controllers
        if self._load_and_start_controllers():
            rospy.loginfo("[ScissorsMonitor] Reconnection complete (restart #%d)",
                          self.restart_count)
        else:
            rospy.logerr("[ScissorsMonitor] Controller setup failed -- "
                         "will retry on next detection")

    # ------------------------------------------------------------------ #
    #  Main
    # ------------------------------------------------------------------ #
    def run(self):
        rospy.loginfo("[ScissorsMonitor] Monitoring active")
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = ScissorsConnectionMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
