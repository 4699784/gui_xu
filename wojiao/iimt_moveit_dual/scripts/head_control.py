#!/usr/bin/env python3

#######################################
##########可以微小的动一动，待调
#######################################

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node("test_head_movement")

client = actionlib.SimpleActionClient("/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
rospy.loginfo("Waiting for head controller action server...")
client.wait_for_server()

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ["Joint_Neck_Pitch_1", "Joint_Neck_Yaw_1"]

point = JointTrajectoryPoint()
# 尝试微小角度变化（单位：弧度，约2.86度 = 0.05 rad）
point.positions = [0.05, -0.03]  # pitch 向上一点点，yaw 向左一点点
point.time_from_start = rospy.Duration(2.0)  # 动作时间可缩短便于快速观察
goal.trajectory.points.append(point)
goal.trajectory.header.stamp = rospy.Time.now()

rospy.loginfo(f"Sending head command: pitch={point.positions[0]:.3f}, yaw={point.positions[1]:.3f}")
client.send_goal(goal)
client.wait_for_result()

result = client.get_result()
if result.error_code != 0:
    rospy.logwarn(f"Goal failed with error code: {result.error_code}, message: '{result.error_string}'")
else:
    rospy.loginfo("Head movement completed successfully.")