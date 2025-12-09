#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node("reset_head")

client = actionlib.SimpleActionClient("/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
client.wait_for_server()

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ["Joint_Neck_Pitch_1", "Joint_Neck_Yaw_1"]  

point = JointTrajectoryPoint()
point.positions = [0, 0]  # 归零位置
point.time_from_start = rospy.Duration(10.0)  
goal.trajectory.points.append(point)
goal.trajectory.header.stamp = rospy.Time.now()

client.send_goal(goal)
client.wait_for_result()
rospy.loginfo("Head reset to zero position.")