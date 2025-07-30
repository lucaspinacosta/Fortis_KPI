#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import select
import sys
import time

import actionlib
import keyboard
#%% Imports
import rospy
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import Imu, NavSatFix
from telehandler_moveit.msg import (MoveToPositionAction,
                                    MoveToPositionActionGoal,
                                    MoveToPositionGoal)

#%% KPI Timer Monitoring    

rospy.init_node('kpi_scripts', anonymous=True, disable_signals=True)

_gpt_topic = '/ugv0/telehandler/gps/fix'
_odometry_topic = '/ugv0/telehandler/odom'
_imu_topic = '/ugv0/telehandler/imu/wit/imu'

# Trajectory Path Planning KPI Timer Monitoring
_display_trajectory_topic = '/ugv0/telehandler/move_group/display_planned_path'
trajectory_data : DisplayTrajectory = None
goal_data : MoveToPositionActionGoal = None

_goal_action_topic = '/ugv0/telehandler/action/MoveToPosition'
_goal_action_client = actionlib.SimpleActionClient(_goal_action_topic, MoveToPositionAction)
rospy.loginfo("Waiting for goal action server...")
_goal_action_client.wait_for_server()
rospy.loginfo("Goal action server found")


_goal_topic = '/ugv0/telehandler/action/MoveToPosition/goal'



def get_display_trajectory_data(msg):
    global trajectory_data
    trajectory_data = msg
    return trajectory_data


def generate_goal_data():
    # {'data': '-0.852'}, {'data': 3.187'}, {'data': '8.5'}
    global goal_data
    goal_data = MoveToPositionGoal()
    goal_data.predefined_pose = ""
    goal_data.target_pose.header.frame_id = "base_link"
    goal_data.target_pose.header.stamp = rospy.Time.now()
    goal_data.target_pose.pose.position.x = -0.852
    goal_data.target_pose.pose.position.y = 3.187
    goal_data.target_pose.pose.position.z = 8.5
    goal_data.target_pose.pose.orientation.x = 0
    goal_data.target_pose.pose.orientation.y = 0
    goal_data.target_pose.pose.orientation.z = 0
    goal_data.target_pose.pose.orientation.w = 1.0
    return goal_data
    
    
    
    
print("Press 'q' + Enter to abort waiting")

_goal = generate_goal_data()
_goal_action_client.send_goal(_goal)       
_time_start = rospy.get_time()

_subs_trajectory = rospy.Subscriber(_display_trajectory_topic, DisplayTrajectory, get_display_trajectory_data)


while trajectory_data is None:
    print("Waiting for trajectory data...")
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    if ready:
        line = sys.stdin.readline().strip()
        if line.lower() == 'q' or line.lower() == "exit" or line.lower() == "logout":
            print("Loop aborted by user")
            raise KeyboardInterrupt
_time_end = rospy.get_time()


total_time = _time_end - _time_start



print(f"Total time: {total_time}")


# %%
