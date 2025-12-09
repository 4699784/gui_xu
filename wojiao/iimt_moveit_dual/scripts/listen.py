#!/usr/bin/env python3

import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryListener:
    def __init__(self):
        self.expected_joint_names = [
           "Joint_ul_l_shoulder_pitch",
            "Joint_ul_l_shoulder_roll",
            "Joint_ul_l_shoulder_yaw",
            "Joint_ul_l_elbow_pitch",
            "Joint_ul_l_wrist_yaw",
            "Joint_ul_l_wrist_pitch",
            "Joint_ul_r_shoulder_pitch",
            "Joint_ul_r_shoulder_roll",
            "Joint_ul_r_shoulder_yaw",
            "Joint_ul_r_elbow_pitch",
            "Joint_ul_r_wrist_yaw",
            "Joint_ul_r_wrist_pitch"
        ]

        # 创建 Action Server
        self.server = actionlib.SimpleActionServer(
            "/execute_trajectory",
            ExecuteTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("Trajectory listener started. Waiting for MoveIt! trajectories...")

    def execute_cb(self, goal):
        trajectory = goal.trajectory.joint_trajectory
        received_names = trajectory.joint_names
        num_left_joints = 6

        left_positions = trajectory.points[:num_left_joints]      
        right_positions = trajectory.points[num_left_joints:] 

        rospy.loginfo("Received trajectory with %d points", len(trajectory.points))
        rospy.loginfo("Received joint names: %s", received_names)

        # self.send_to_arms(trajectory)

        self.server.set_succeeded()
        rospy.loginfo("Trajectory processing completed.")

    def send_to_arms(self, trajectory):
        pass


if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    listener = TrajectoryListener()
    rospy.spin()