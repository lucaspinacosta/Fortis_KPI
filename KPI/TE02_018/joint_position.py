#!/usr/bin/env python3
import math
import sys

import actionlib
import rospy
from moveit_commander import (MoveGroupCommander, PlanningSceneInterface,
                              RobotCommander, RobotState, roscpp_initialize,
                              roscpp_shutdown)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from telehandler_moveit.msg import (MoveToPositionAction, MoveToPositionGoal,
                                    MoveToPositionResult)

joint_state = None
low_level_joints = None


def euclidean_distance(p1, p2):
    return math.sqrt(sum([(a - b) ** 2 for a, b in zip(p1, p2)]))

def state_cb(msg):

    global joint_state
    joint_state = [msg.position[0], msg.position[1] + msg.position[2] + msg.position[3] + 6.390909194946289, msg.position[5]]
    # Process the RobotState message
    # rospy.loginfo(f"Received Positions {[msg.position[0], msg.position[2]+ (msg.position[3]+ msg.position[4])*10, msg.position[5]]}")


def low_level_joints_cb(msg):
    global low_level_joints
    low_level_joints = msg.data
    # Process the Float32MultiArray message
    # rospy.loginfo(f"Received low level joints: {msg.data}")


def main():
    rospy.loginfo("Initializing MoveIt Commander...")

    move_group = MoveGroupCommander("Arm_Group",
                                    robot_description="/ugv0/telehandler/robot_description",
                                    ns="/ugv0/telehandler",
                                    wait_for_servers=5)

    # List of desired end-effector positions (set your KPI test positions here)
    desired_positions = [
        [7.3, 0.0, 1.5],
        [7.5, 0.0, 1.4],
        [7.7, 0.0, 1.6],
        # add more as needed...
    ]

    _sub = rospy.Subscriber(
        "/ugv0/telehandler/joint_states", JointState, state_cb)
    _low_level_sub = rospy.Subscriber(
        "/ugv0/telehandler/joints", Float32MultiArray, low_level_joints_cb)
    
    _action_goal = actionlib.SimpleActionClient(
        '/ugv0/telehandler/action/MoveToPosition', MoveToPositionAction)
    _action_goal.wait_for_server()
    
    rate = rospy.Rate(20)  # 20 Hz

    pass_samples = 0
    total_samples = 0

    rospy.loginfo("Ready to start KPI validation test.")

    for desired_position in desired_positions:
        input("Press Enter to send next goal to position: {}".format(desired_position))
        rospy.loginfo(f"Sending goal to position: {desired_position}")
        
        goal = MoveToPositionGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.pose.position.x = desired_position[0]
        goal.target_pose.pose.position.y = desired_position[1]
        goal.target_pose.pose.position.z = desired_position[2]
        goal.target_pose.pose.orientation.w = 1.0
        

        _action_goal.send_goal(goal)
        _action_goal.wait_for_result()

        # After execution, get current end-effector position
        current_pose = move_group.get_current_pose().pose
        actual_position = [current_pose.position.x,
                           current_pose.position.y, current_pose.position.z]

        error = euclidean_distance(actual_position, desired_position)

        if error <= 0.15:
            pass_samples += 1
            rospy.loginfo(f"KPI Pass: Position error {error:.4f} m")
        else:
            rospy.logwarn(f"KPI Fail: Position error {error:.4f} m")

        total_samples += 1
        

    # After all points tested, print the success rate summary
    success_rate = (pass_samples / total_samples) * \
        100 if total_samples > 0 else 0
    rospy.loginfo(f"KPI Validation completed.")
    rospy.loginfo(f"Total tests: {total_samples}")
    rospy.loginfo(f"Passed tests: {pass_samples}")
    rospy.loginfo(f"Success rate: {success_rate:.2f}%")


if __name__ == '__main__':
    rospy.init_node('move_group_commander_example',
                    anonymous=True, disable_signals=True, log_level=rospy.DEBUG)
    roscpp_initialize(sys.argv)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
        sys.exit(1)
    finally:
        roscpp_shutdown()
        print("ROS shutdown complete.")
        sys.exit(0)
