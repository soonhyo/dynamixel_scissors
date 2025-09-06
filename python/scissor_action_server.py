#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import actionlib_msgs.msg

from dynamixel_scissors.msg import ScissorControlAction, ScissorControlGoal, ScissorControlResult, ScissorControlFeedback

class ScissorActionServer:
    def __init__(self):
        rospy.init_node('scissor_action_server', anonymous=True)
        
        self.pub = rospy.Publisher(
            '/sample_robot/position_joint_trajectory_controller/follow_joint_trajectory/goal',
            FollowJointTrajectoryActionGoal,
            queue_size=10
        )
        
        self.joint_sub = rospy.Subscriber(
            '/sample_robot/joint_states',
            JointState,
            self.joint_state_callback,
            queue_size=1
        )
        
        self.server = actionlib.SimpleActionServer(
            'scissor_control',
            ScissorControlAction,
            self.execute_callback,
            False
        )
        
        # Core state variables
        self.current_position = 0.0
        self.center_position = 0.0
        self.max_position = 0.50
        self.min_position = -3.14
        self.joint_state_received = False
        self.target_joint = 'sample_joint'
        self.last_command_time = rospy.Time.now()
        self.min_command_interval = 0.05  # 50ms between commands
        self.goal_id_counter = 0
        
        # Safety parameters
        self.max_effort = 1.0
        self.current_effort = 0.0
        self.safety_open_distance = 0.2
        self.safety_active = False
        self.last_safety_time = rospy.Time.now()
        self.safety_cooldown = 2.0
        
        rospy.loginfo("Scissor Action Server Started")
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
            
        self.server.start()
        rospy.loginfo("Action server ready to accept goals")
    
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
            if self.safety_active and self.current_effort < self.max_effort * 0.8:
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
        self.publish_trajectory(target_pos, duration=0.5)
    
    def cancel_current_trajectory(self):
        """Cancel current trajectory by sending goal with current position"""
        rospy.loginfo("Cancelling current trajectory")
        self.publish_trajectory(self.current_position, duration=0.1)
    
    def publish_trajectory(self, position, duration=0.3):
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
        
        msg.goal.trajectory.joint_names = ['sample_joint']
        
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
    
    def execute_callback(self, goal):
        """Execute action server callback"""
        rospy.loginfo(f"Received goal: command='{goal.command}', position={goal.position}, duration={goal.duration}")
        
        # Create feedback message
        feedback = ScissorControlFeedback()
        result = ScissorControlResult()
        
        # Set default duration if not specified
        duration = goal.duration if goal.duration > 0 else 1.0
        
        try:
            success = False
            message = ""
            
            if goal.command == "open":
                # Incremental open
                increment = 0.05
                target_pos = min(self.current_position + increment, self.max_position)
                success = self.publish_trajectory(target_pos, duration)
                message = f"Open command executed to position {target_pos:.3f}"
                
            elif goal.command == "close":
                # Incremental close
                increment = 0.05
                target_pos = max(self.current_position - increment, self.min_position)
                success = self.publish_trajectory(target_pos, duration)
                message = f"Close command executed to position {target_pos:.3f}"
                
            elif goal.command == "full_open":
                target_pos = self.max_position
                success = self.publish_trajectory(target_pos, duration)
                message = f"Full open executed to position {target_pos:.3f}"
                
            elif goal.command == "full_close":
                target_pos = self.min_position
                success = self.publish_trajectory(target_pos, duration)
                message = f"Full close executed to position {target_pos:.3f}"
                
            elif goal.command == "set_position":
                target_pos = goal.position
                if target_pos > self.max_position or target_pos < self.min_position:
                    success = False
                    message = f"Position {target_pos:.3f} is outside safe limits [{self.min_position:.2f}, {self.max_position:.2f}]"
                else:
                    success = self.publish_trajectory(target_pos, duration)
                    message = f"Set position executed to {target_pos:.3f}"
                    
            elif goal.command == "toggle":
                # Toggle between open and close based on center position
                if self.current_position > self.center_position:
                    target_pos = self.min_position
                    success = self.publish_trajectory(target_pos, duration)
                    message = f"Toggle: close to position {target_pos:.3f}"
                else:
                    target_pos = self.max_position
                    success = self.publish_trajectory(target_pos, duration)
                    message = f"Toggle: open to position {target_pos:.3f}"
                    
            elif goal.command == "reset_center":
                target_pos = self.center_position
                success = self.publish_trajectory(target_pos, duration)
                message = f"Reset to center position {target_pos:.3f}"
                
            elif goal.command == "set_center":
                self.center_position = self.current_position
                success = True
                message = f"Set new center position to {self.center_position:.3f}"
                
            else:
                success = False
                message = f"Unknown command: {goal.command}"
            
            if not success:
                rospy.logwarn(message)
                result.success = False
                result.message = message
                result.final_position = self.current_position
                self.server.set_aborted(result)
                return
            
            # Wait for motion to complete and provide feedback
            start_time = rospy.Time.now()
            rate = rospy.Rate(10)  # 10 Hz feedback
            
            while (rospy.Time.now() - start_time).to_sec() < duration + 0.5:  # Add small buffer
                if self.server.is_preempt_requested():
                    rospy.loginfo("Goal preempted")
                    self.server.set_preempted()
                    return
                    
                # Publish feedback
                feedback.current_position = self.current_position
                feedback.current_effort = self.current_effort
                feedback.safety_active = self.safety_active
                feedback.status_message = f"Moving to target position"
                self.server.publish_feedback(feedback)
                
                rate.sleep()
            
            # Success
            result.success = True
            result.message = message
            result.final_position = self.current_position
            
            rospy.loginfo(f"Goal completed successfully: {message}")
            self.server.set_succeeded(result)
            
        except Exception as e:
            rospy.logerr(f"Error executing goal: {str(e)}")
            result.success = False
            result.message = f"Error: {str(e)}"
            result.final_position = self.current_position
            self.server.set_aborted(result)

if __name__ == '__main__':
    try:
        server = ScissorActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass