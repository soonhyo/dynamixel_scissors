#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import actionlib_msgs.msg
from scissor_config import ScissorConfig

class ScissorControlNode:
    def __init__(self, config_file=None):
        rospy.init_node('scissor_control_node', anonymous=True)

        # Load configuration
        self.config = ScissorConfig(config_file)
        self.config.print_config_summary()

        self.pub = rospy.Publisher(
            self.config.get_trajectory_goal_topic(),
            FollowJointTrajectoryActionGoal,
            queue_size=10
        )

        self.joint_sub = rospy.Subscriber(
            self.config.get_joint_states_topic(),
            JointState,
            self.joint_state_callback,
            queue_size=1
        )

        self.current_position = 0.0
        self.center_position = 0.0
        self.position_increment = self.config.get_position_increment()
        self.max_position = self.config.get_max_position()
        self.min_position = self.config.get_min_position()
        self.joint_state_received = False
        self.target_joint = self.config.get_joint_name()
        self.last_command_time = rospy.Time.now()
        self.min_command_interval = self.config.get_min_command_interval()
        self.goal_id_counter = 0

        # Safety parameters
        self.max_effort = self.config.get_max_effort()
        self.current_effort = 0.0
        self.safety_open_distance = self.config.get_safety_open_distance()
        self.safety_active = False
        self.last_safety_time = rospy.Time.now()
        self.safety_cooldown = self.config.get_safety_cooldown()
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        rospy.loginfo("Scissor Control Node Started")
        rospy.loginfo("Waiting for initial joint state...")
        
        # Wait for initial joint state
        timeout = 5.0
        start_time = rospy.Time.now()
        while not self.joint_state_received and (rospy.Time.now() - start_time).to_sec() < timeout:
            rospy.sleep(0.1)
            
        if not self.joint_state_received:
            rospy.logwarn("No joint state received after 5 seconds, using default position 0.0")
        else:
            rospy.loginfo(f"Successfully received initial joint state: {self.current_position:.6f} rad")
        rospy.loginfo("Controls:")
        rospy.loginfo("  w/s: Open/Close scissor")
        rospy.loginfo("  a/d: Decrease/Increase increment size")
        rospy.loginfo("  r: Reset to center position")
        rospy.loginfo("  c: Set current position as new center")
        rospy.loginfo("  o: Full open (max position)")
        rospy.loginfo("  p: Full close (min position)")
        rospy.loginfo("  t: Toggle scissor (open<->close)")
        rospy.loginfo("  u/j: Increase/Decrease max effort threshold")
        rospy.loginfo("  k/m: Increase/Decrease safety open distance")
        rospy.loginfo("  i: Show current status info")
        rospy.loginfo("  q: Quit")
        rospy.loginfo(f"Current position: {self.current_position:.2f}")
        rospy.loginfo(f"Center position: {self.center_position:.2f}")
        rospy.loginfo(f"Position increment: {self.position_increment:.2f}")
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def check_safety(self):
        """Check if effort exceeds safety limit and trigger protective action"""
        if self.current_effort > self.max_effort:
            current_time = rospy.Time.now()
            # Only trigger safety if not in cooldown period
            if (current_time - self.last_safety_time).to_sec() > self.safety_cooldown:
                rospy.logwarn(f"SAFETY TRIGGERED! Effort: {self.current_effort:.3f} > {self.max_effort:.3f}")
                self.trigger_safety_open()
                self.last_safety_time = current_time
                self.safety_active = True
        else:
            # Reset safety flag when effort is normal
            threshold = self.max_effort * self.config.get_effort_normal_threshold()
            if self.safety_active and self.current_effort < threshold:
                rospy.loginfo("Safety condition cleared - effort back to normal")
                self.safety_active = False
    
    def trigger_safety_open(self):
        """Emergency opening when high effort detected"""
        # Cancel current trajectory by sending a stop command
        self.cancel_current_trajectory()
        
        # Open by safety distance
        target_pos = min(self.current_position + self.safety_open_distance, self.max_position)
        rospy.logwarn(f"Safety opening: {self.current_position:.3f} -> {target_pos:.3f}")
        
        # Use shorter duration for emergency opening
        self.publish_trajectory(target_pos, duration=self.config.get_emergency_duration())
    
    def cancel_current_trajectory(self):
        """Cancel current trajectory by sending goal with current position"""
        rospy.loginfo("Cancelling current trajectory")
        self.publish_trajectory(self.current_position, duration=0.1)
    
    def joint_state_callback(self, msg):
        try:
            if self.target_joint in msg.name:
                joint_index = msg.name.index(self.target_joint)
                new_position = msg.position[joint_index]
                if not self.joint_state_received:
                    self.current_position = new_position
                    self.center_position = new_position
                    rospy.loginfo(f"Initial position from joint state: {new_position:.2f}")
                    self.joint_state_received = True
                else:
                    self.current_position = new_position
                    
                # Update effort and check safety
                if len(msg.effort) > joint_index:
                    self.current_effort = abs(msg.effort[joint_index])
                    self.check_safety()
        except (ValueError, IndexError) as e:
            if not self.joint_state_received:
                rospy.logwarn(f"Could not find joint '{self.target_joint}' in joint states")
    
    def publish_trajectory(self, position, duration=None):
        if duration is None:
            duration = self.config.get_default_duration()
        # Safety check
        if position > self.max_position or position < self.min_position:
            rospy.logerr(f"Position {position:.3f} is outside safe limits [{self.min_position:.2f}, {self.max_position:.2f}]")
            return False
            
        # Rate limiting for smooth operation
        current_time = rospy.Time.now()
        if (current_time - self.last_command_time).to_sec() < self.min_command_interval:
            return False
        self.last_command_time = current_time
        
        # Increment goal ID for trajectory replacement
        self.goal_id_counter += 1
        msg = FollowJointTrajectoryActionGoal()
        
        # Header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        
        # Goal ID for trajectory replacement
        msg.goal_id = actionlib_msgs.msg.GoalID()
        msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = f'scissor_goal_{self.goal_id_counter}'
        
        # Trajectory
        msg.goal.trajectory.header = Header()
        msg.goal.trajectory.header.stamp = rospy.Time.now()
        msg.goal.trajectory.header.frame_id = ''
        
        msg.goal.trajectory.joint_names = [self.target_joint]
        
        # Create smooth trajectory with intermediate points
        current_pos = self.current_position
        pos_diff = position - current_pos
        
        # Add intermediate point for smoother motion
        mid_time = duration * 0.6
        mid_pos = current_pos + pos_diff * 0.6
        mid_vel = pos_diff / duration
        
        # Intermediate point
        mid_point = JointTrajectoryPoint()
        mid_point.positions = [mid_pos]
        mid_point.velocities = [mid_vel]
        mid_point.accelerations = [0.0]
        mid_point.effort = [0.0]
        mid_point.time_from_start = rospy.Duration(mid_time)
        
        # Final point
        final_point = JointTrajectoryPoint()
        final_point.positions = [position]
        final_point.velocities = [0.0]  # Come to rest
        final_point.accelerations = [0.0]
        final_point.effort = [0.0]
        final_point.time_from_start = rospy.Duration(duration)
        
        msg.goal.trajectory.points = [mid_point, final_point]
        
        # Tolerances (empty for default)
        msg.goal.path_tolerance = []
        msg.goal.goal_tolerance = []
        msg.goal.goal_time_tolerance = rospy.Duration(0)
        
        self.pub.publish(msg)
        rospy.loginfo(f"Published position: {position:.2f}")
        return True
    
    def full_open(self, duration=None):
        """Open scissor fully to configured open position"""
        if duration is None:
            duration = self.config.get_full_motion_duration()
        target_pos = self.config.get_open_position()
        if self.publish_trajectory(target_pos, duration):
            self.current_position = target_pos
            precision = self.config.get_position_precision()
            rospy.loginfo(f"Full open executed: {target_pos:.{precision}f} rad")
            return True
        return False
    
    def full_close(self, duration=None):
        """Close scissor fully to configured close position"""
        if duration is None:
            duration = self.config.get_full_motion_duration()
        target_pos = self.config.get_close_position()
        if self.publish_trajectory(target_pos, duration):
            self.current_position = target_pos
            precision = self.config.get_position_precision()
            rospy.loginfo(f"Full close executed: {target_pos:.{precision}f} rad")
            return True
        return False
    
    def toggle_scissor(self, duration=None):
        """Toggle scissor state (open->close or close->open)"""
        if duration is None:
            duration = self.config.get_full_motion_duration()

        open_pos = self.config.get_open_position()
        close_pos = self.config.get_close_position()

        # Calculate distance to open and close positions
        dist_to_open = abs(self.current_position - open_pos)
        dist_to_close = abs(self.current_position - close_pos)

        # Go to the position that's farther away
        if dist_to_open > dist_to_close:
            return self.full_open(duration)
        else:
            return self.full_close(duration)
    
    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == 'q':
                    break
                elif key == 'w':  # Open scissor
                    new_pos = self.current_position + self.position_increment
                    if new_pos <= self.max_position:
                        self.current_position = new_pos
                        self.publish_trajectory(self.current_position)
                    else:
                        rospy.logwarn("Maximum position reached!")
                        
                elif key == 's':  # Close scissor
                    new_pos = self.current_position - self.position_increment
                    if new_pos >= self.min_position:
                        self.current_position = new_pos
                        self.publish_trajectory(self.current_position)
                    else:
                        rospy.logwarn("Minimum position reached!")
                        
                elif key == 'a':  # Decrease increment
                    self.position_increment = max(0.01, self.position_increment - 0.01)
                    rospy.loginfo(f"Position increment: {self.position_increment:.2f}")
                    
                elif key == 'd':  # Increase increment
                    self.position_increment = min(0.2, self.position_increment + 0.01)
                    rospy.loginfo(f"Position increment: {self.position_increment:.2f}")
                    
                elif key == 'r':  # Reset to center
                    self.current_position = self.center_position
                    self.publish_trajectory(self.current_position)
                    rospy.loginfo(f"Reset to center position: {self.center_position:.2f}")
                    
                elif key == 'c':  # Set current as center
                    self.center_position = self.current_position
                    rospy.loginfo(f"Set new center position: {self.center_position:.2f}")
                    
                elif key == 'o':  # Full open
                    self.full_open()
                    
                elif key == 'p':  # Full close
                    self.full_close()
                    
                elif key == 't':  # Toggle scissor
                    self.toggle_scissor()
                    
                elif key == 'u':  # Increase max effort
                    self.max_effort = min(2.0, self.max_effort + 0.1)
                    rospy.loginfo(f"Max effort threshold: {self.max_effort:.2f}")
                    
                elif key == 'j':  # Decrease max effort
                    self.max_effort = max(0.1, self.max_effort - 0.1)
                    rospy.loginfo(f"Max effort threshold: {self.max_effort:.2f}")
                    
                elif key == 'k':  # Increase safety distance
                    self.safety_open_distance = min(1.0, self.safety_open_distance + 0.05)
                    rospy.loginfo(f"Safety open distance: {self.safety_open_distance:.2f}")
                    
                elif key == 'm':  # Decrease safety distance
                    self.safety_open_distance = max(0.05, self.safety_open_distance - 0.05)
                    rospy.loginfo(f"Safety open distance: {self.safety_open_distance:.2f}")
                    
                elif key == 'i':  # Show status info
                    pos_prec = self.config.get_position_precision()
                    eff_prec = self.config.get_effort_precision()
                    rospy.loginfo("=== Status Info ===")
                    rospy.loginfo(f"Current position: {self.current_position:.{pos_prec}f} rad")
                    rospy.loginfo(f"Center position: {self.center_position:.{pos_prec}f} rad")
                    rospy.loginfo(f"Position increment: {self.position_increment:.{pos_prec}f} rad")
                    rospy.loginfo(f"Position limits: [{self.min_position:.2f}, {self.max_position:.2f}] rad")
                    rospy.loginfo(f"Open position: {self.config.get_open_position():.{pos_prec}f} rad")
                    rospy.loginfo(f"Close position: {self.config.get_close_position():.{pos_prec}f} rad")
                    rospy.loginfo(f"Open is max: {self.config.is_open_max()}")
                    rospy.loginfo(f"Current effort: {self.current_effort:.{eff_prec}f}")
                    rospy.loginfo(f"Max effort threshold: {self.max_effort:.2f}")
                    rospy.loginfo(f"Safety open distance: {self.safety_open_distance:.2f}")
                    rospy.loginfo(f"Safety active: {self.safety_active}")
                    rospy.loginfo(f"Joint state received: {self.joint_state_received}")
                    rospy.loginfo("=================")
                    
                elif key == '\x03':  # Ctrl+C
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            rospy.loginfo("Scissor Control Node Shutting Down")

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Scissor Control Node')
    parser.add_argument('--config', '-c', type=str, default=None,
                        help='Path to configuration YAML file')

    # Parse only known args to avoid conflicts with ROS arguments
    args, unknown = parser.parse_known_args()

    try:
        node = ScissorControlNode(config_file=args.config)
        node.run()
    except rospy.ROSInterruptException:
        pass
