#!/usr/bin/env python3

import rospy
import actionlib
from dynamixel_scissors.msg import ScissorControlAction, ScissorControlGoal

def test_scissor_client():
    rospy.init_node('test_scissor_client')
    
    # Create the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('scissor_control', ScissorControlAction)
    
    # Wait for the server to start up and declare that it is ready to receive goals
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to action server")
    
    def send_command(command, position=0.0, duration=1.0):
        """Send a command to the scissor action server"""
        goal = ScissorControlGoal()
        goal.command = command
        goal.position = position
        goal.duration = duration
        
        rospy.loginfo(f"Sending command: {command} (pos: {position}, duration: {duration})")
        
        # Send the goal to the action server
        client.send_goal(goal)
        
        # Wait for the server to finish performing the action
        client.wait_for_result()
        
        # Get the result
        result = client.get_result()
        state = client.get_state()
        
        rospy.loginfo(f"Action completed. State: {state}")
        rospy.loginfo(f"Result: Success={result.success}, Message='{result.message}', Final Position={result.final_position:.3f}")
        
        return result.success
    
    try:
        # Test various commands
        rospy.loginfo("=== Testing Scissor Action Server ===")
        
        # Test set center
        rospy.loginfo("\n1. Setting current position as center...")
        send_command("set_center")
        rospy.sleep(1)
        
        # Test full open
        rospy.loginfo("\n2. Testing full open...")
        send_command("full_open", duration=2.0)
        rospy.sleep(3)
        
        # Test full close
        rospy.loginfo("\n3. Testing full close...")
        send_command("full_close", duration=2.0)
        rospy.sleep(3)
        
        # Test reset to center
        rospy.loginfo("\n4. Testing reset to center...")
        send_command("reset_center", duration=1.5)
        rospy.sleep(2)
        
        # Test incremental open
        rospy.loginfo("\n5. Testing incremental open...")
        for i in range(3):
            send_command("open", duration=0.5)
            rospy.sleep(1)
        
        # Test incremental close
        rospy.loginfo("\n6. Testing incremental close...")
        for i in range(3):
            send_command("close", duration=0.5)
            rospy.sleep(1)
        
        # Test set position
        rospy.loginfo("\n7. Testing set specific position (0.2)...")
        send_command("set_position", position=0.2, duration=1.0)
        rospy.sleep(2)
        
        # Test toggle
        rospy.loginfo("\n8. Testing toggle (should close)...")
        send_command("toggle", duration=1.5)
        rospy.sleep(2)
        
        rospy.loginfo("\n9. Testing toggle again (should open)...")
        send_command("toggle", duration=1.5)
        rospy.sleep(2)
        
        # Return to center
        rospy.loginfo("\n10. Returning to center...")
        send_command("reset_center", duration=1.0)
        
        rospy.loginfo("\n=== All tests completed successfully ===")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {str(e)}")

if __name__ == '__main__':
    test_scissor_client()