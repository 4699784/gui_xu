#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node("reset_arms")

left_joint_names = [
    "Joint_ul_l_shoulder_pitch",
    "Joint_ul_l_shoulder_roll",
    "Joint_ul_l_shoulder_yaw",
    "Joint_ul_l_elbow_pitch",
    "Joint_ul_l_wrist_yaw",
    "Joint_ul_l_wrist_pitch"
]

right_joint_names = [
    "Joint_ul_r_shoulder_pitch",
    "Joint_ul_r_shoulder_roll",
    "Joint_ul_r_shoulder_yaw",
    "Joint_ul_r_elbow_pitch",
    "Joint_ul_r_wrist_yaw",
    "Joint_ul_r_wrist_pitch"
]

# 创建 Action 客户端
left_client = actionlib.SimpleActionClient("/left_arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
right_client = actionlib.SimpleActionClient("/right_arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

rospy.loginfo("Waiting for left and right arm controllers...")
left_client.wait_for_server()
right_client.wait_for_server()

# 构建归零轨迹（所有关节设为 0.0）
def create_zero_goal(joint_names):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = [0.0] * len(joint_names)
    point.time_from_start = rospy.Duration(10.0)  # 10秒内完成
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()
    return goal

# 发送目标
left_client.send_goal(create_zero_goal(left_joint_names))
right_client.send_goal(create_zero_goal(right_joint_names))

# 等待结果（可选）
left_client.wait_for_result()
right_client.wait_for_result()

rospy.loginfo("Both arms have been reset to zero position.")