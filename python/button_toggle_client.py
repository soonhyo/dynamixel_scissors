#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String
from dynamixel_scissors.msg import ScissorControlAction, ScissorControlGoal

class ButtonToggleClient:
    def __init__(self):
        rospy.init_node('button_toggle_client')
        
        # Create the SimpleActionClient for scissor control
        self.client = actionlib.SimpleActionClient('scissor_control', ScissorControlAction)
        
        # Wait for the action server to start
        rospy.loginfo("Waiting for scissor control action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to scissor control action server")
        
        # Subscribe to Oculus controller button topic
        self.joy_subscriber = rospy.subscribe("/oculus/left_controller/joy", String, self.command_pose_callback)
        
        rospy.loginfo("Button toggle client ready. Press button_one to toggle scissor.")
        
    def command_pose_callback(self, msg):
        command_pose_data = msg.data
        
        if command_pose_data == "button/button_one":
            rospy.loginfo("Button one pressed - toggling scissor")
            self.send_toggle_command()
    
    def send_toggle_command(self):
        # Create goal message
        goal = ScissorControlGoal()
        goal.command = "toggle"
        goal.duration = 1.5
        
        # Send the goal to the action server
        self.client.send_goal(goal)
        
        # Wait for the result (non-blocking in callback)
        # We could add a callback here if needed, but for simplicity we'll just send and forget
        rospy.loginfo("Toggle command sent to scissor control server")

def main():
    try:
        client = ButtonToggleClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Button toggle client shutting down")

if __name__ == '__main__':
    main()