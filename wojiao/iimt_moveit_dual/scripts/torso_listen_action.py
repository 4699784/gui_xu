#!/usr/bin/env python3

import rospy
import actionlib
import moveit_commander
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray  


class TorsoTrajectoryListener:
    def __init__(self):
        moveit_commander.roscpp_initialize([])

        self.torso_joint_names = [
            "Joint_DownLimb_Pitch_1",
            "Joint_DownLimb_Pitch_2",
            "Joint_DownLimb_Pitch_3",
            "Joint_DownLimb_Yaw_1"
        ]

        try:
            self.torso_group = moveit_commander.MoveGroupCommander("torso")
            self.torso_group.set_planning_time(10.0)
            self.torso_group.set_goal_position_tolerance(0.5)
        except Exception as e:
            rospy.logerr(f"Failed to initialize MoveGroupCommander for 'torso': {e}")
            self.torso_group = None

        #torso 轨迹客户端
        self.torso_client = actionlib.SimpleActionClient(
            '/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        if not self.torso_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Torso action server not available!")
            return
        rospy.loginfo("Connected to torso_controller action server.")

        # torso 目标关节订阅
        self.target_sub = rospy.Subscriber(
            '/torso_target_joints', 
            Float64MultiArray, 
            self.torso_joint_target_callback
        )
        rospy.loginfo("Torso trajectory listener started. Waiting for /torso_target_joints...")

    def torso_joint_target_callback(self, msg):
        if len(msg.data) != len(self.torso_joint_names):
            rospy.logerr(f"Expected {len(self.torso_joint_names)} joint values, got {len(msg.data)}")
            return

        if self.torso_group is None:
            rospy.logerr("Torso MoveGroup not initialized")
            return

        rospy.loginfo(f"Received torso target joints: {msg.data}")

        # 设置关节目标
        self.torso_group.set_joint_value_target(msg.data)

        # 规划
        success, plan_trajectory, planning_time, error_code = self.torso_group.plan()

        if success and len(plan_trajectory.joint_trajectory.points) > 0:
            rospy.loginfo("Torso planning succeeded, executing...")
            self.execute_cb_torso(plan_trajectory.joint_trajectory)
        else:
            rospy.logwarn("Torso planning failed or returned empty trajectory")

    def execute_cb_torso(self, joint_trajectory):
        # 创建发送给控制器的轨迹（确保 joint_names 顺序正确）
        traj_to_send = JointTrajectory()
        traj_to_send.joint_names = self.torso_joint_names  # 必须与 controller 一致
        traj_to_send.header.stamp = rospy.Time.now()  # 立即执行
        traj_to_send.points = joint_trajectory.points

        # 构造 action goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj_to_send
        goal.goal_time_tolerance = rospy.Duration(1.0)

        # 发送并等待结果
        self.torso_client.send_goal(goal)
        if self.torso_client.wait_for_result(rospy.Duration(10.0)):
            result = self.torso_client.get_result()
            if result and result.error_code == 0:
                rospy.loginfo("Torso trajectory execution succeeded.")
            else:
                rospy.logerr(f"Torso execution failed with error code: {result.error_code if result else 'unknown'}")
        else:
            rospy.logerr("Torso execution timed out!")


if __name__ == '__main__':
    rospy.init_node('torso_trajectory_listener', anonymous=True)
    listener = TorsoTrajectoryListener()
    rospy.spin()