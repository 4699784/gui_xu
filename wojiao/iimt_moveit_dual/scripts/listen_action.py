#!/usr/bin/env python3

import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal,ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
#from robot_control import HeadController,LeftHandController

class TrajectoryListener:
    def __init__(self):
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
        self.all_joint_names = self.left_joint_names + self.right_joint_names

        #left and right action serve
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

        # 创建 Action Server
        self.server = actionlib.SimpleActionServer(
            "/execute_trajectory",
            ExecuteTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("Trajectory listener started. Waiting for MoveIt! trajectories...")
        self.goal_pub = rospy.Publisher('/recorded_moveit_trajectory',JointTrajectory,queue_size=10)

    def execute_cb(self, goal):
        traj = goal.trajectory.joint_trajectory

        self.goal_pub.publish(traj)

        left_traj = JointTrajectory()
        left_traj.joint_names = self.left_joint_names
        left_traj.header = traj.header

        right_traj = JointTrajectory()
        right_traj.joint_names = self.right_joint_names
        right_traj.header = traj.header

        for pt in traj.points:
            left_pt = JointTrajectoryPoint()
            left_pt.positions = pt.positions[:6]
            left_pt.velocities = pt.velocities[:6] if pt.velocities else []
            left_pt.accelerations = pt.accelerations[:6] if pt.accelerations else []
            left_pt.time_from_start = pt.time_from_start
            left_traj.points.append(left_pt)

            right_pt = JointTrajectoryPoint()
            right_pt.positions = pt.positions[6:]
            right_pt.velocities = pt.velocities[6:] if pt.velocities else []
            right_pt.accelerations = pt.accelerations[6:] if pt.accelerations else []
            right_pt.time_from_start = pt.time_from_start
            right_traj.points.append(right_pt)

        left_goal = FollowJointTrajectoryGoal(trajectory=left_traj)
        right_goal = FollowJointTrajectoryGoal(trajectory=right_traj)

        self.left_client.send_goal(left_goal)
        self.right_client.send_goal(right_goal)
        
        #wait for accomplish
        left_done = self.left_client.wait_for_result(rospy.Duration(10.0)) 
        right_done = self.right_client.wait_for_result(rospy.Duration(10.0))

        # head_control = HeadController()
        # head_control.send_cmd([0.7, 0])

        # left_hand_control = LeftHandController()
        # left_hand_control.send_cmd([1000, 1000, 1000, 1000, 1000, 0])

        self.server.set_succeeded()
        rospy.loginfo("Trajectory processing completed.")

    def send_to_arms(self, trajectory):
        pass

if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    listener = TrajectoryListener()
    rospy.spin()