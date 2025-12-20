#!/usr/bin/env python3

import rospy
import actionlib
import moveit_commander
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Point, Pose

class TrajectoryListener:
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        
        self.left_joint_names = [
            "Joint_ul_l_shoulder_pitch",
            "Joint_ul_l_shoulder_roll", 
            "Joint_ul_l_shoulder_yaw",
            "Joint_ul_l_elbow_pitch",
            "Joint_ul_l_wrist_yaw",
            "Joint_ul_l_wrist_pitch"
        ]
        self.right_joint_names = [
            "Joint_ul_r_shoulder_pitch",
            "Joint_ul_r_shoulder_roll",
            "Joint_ul_r_shoulder_yaw", 
            "Joint_ul_r_elbow_pitch",
            "Joint_ul_r_wrist_yaw",
            "Joint_ul_r_wrist_pitch"
        ]

        try:
            self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
            self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
            
            self.left_arm.set_planning_time(10.0)
            self.right_arm.set_planning_time(10.0)
            self.left_arm.set_goal_position_tolerance(0.5)
            self.right_arm.set_goal_position_tolerance(0.01)
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize MoveGroupCommander: {e}")
            self.left_arm = None
            self.right_arm = None

        self.left_client = actionlib.SimpleActionClient(
            '/left_arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.right_client = actionlib.SimpleActionClient(
            '/right_arm_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        self.left_client.wait_for_server()
        self.right_client.wait_for_server()
        rospy.loginfo("Connected to left/right arm action servers.")
        
        # subscribe position and move to target position
        self.position_sub = rospy.Subscriber('/target_position', Point, self.move_to_position_callback)
        
        rospy.loginfo("Trajectory listener started. Waiting for target positions...")

    def move_to_position_callback(self, point_msg):
        rospy.loginfo(f"Received target position: ({point_msg.x}, {point_msg.y}, {point_msg.z})")
        
        if self.left_arm is None:
            rospy.logerr("MoveIt commander not initialized")
            return
        
        # 获取当前方向，保持方向不变
        current_pose = self.left_arm.get_current_pose().pose
        
        # 设置目标位姿
        self.left_arm.set_position_target([point_msg.x, point_msg.y, point_msg.z])
        
        # 规划并执行
        #规划的元组
        success, plan_trajectory, planning_time, error_code = self.left_arm.plan()
        # plan = self.left_arm.plan()
        if success:
            rospy.loginfo("Planning succeeded")
            self.execute_cb_left(plan_trajectory)
        else:
            rospy.logwarn("Planning failed or returned empty trajectory")
    #tarjectory execute
    def execute_cb_left(self, goal):
        traj = goal.joint_trajectory

        # 创建左臂轨迹（直接复用，无需拆分）
        left_traj = JointTrajectory()
        left_traj.joint_names = self.left_joint_names
        left_traj.header = traj.header
        left_traj.points = traj.points  # 直接赋值所有点

        # 构造 Action Goal
        left_goal = FollowJointTrajectoryGoal(trajectory=left_traj)

        # 发送目标并等待结果
        self.left_client.send_goal(left_goal)
        if self.left_client.wait_for_result(rospy.Duration(10.0)):
            rospy.loginfo("Left arm trajectory execution completed.")
        else:
            rospy.logerr("Left arm execution timed out!")

if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    listener = TrajectoryListener()
    rospy.spin()