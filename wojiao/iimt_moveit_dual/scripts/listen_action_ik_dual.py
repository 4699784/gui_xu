#!/usr/bin/env python3

import rospy
import actionlib
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Point

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
            self.left_arm.set_goal_position_tolerance(0.02)
            self.right_arm.set_goal_position_tolerance(0.02)
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize MoveGroupCommander: {e}")
            self.left_arm = None
            self.right_arm = None

        # Action clients for trajectory execution
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

        # Subscribers for left and right targets
        self.left_sub = rospy.Subscriber('/left/target_position', Point, self.move_to_left_position_callback)
        self.right_sub = rospy.Subscriber('/right/target_position', Point, self.move_to_right_position_callback)

        rospy.loginfo("Trajectory listener started. Waiting for /left/target_position and /right/target_position ...")


    def move_to_left_position_callback(self, point_msg):
        rospy.loginfo(f"[Left] Received target position: ({point_msg.x:.3f}, {point_msg.y:.3f}, {point_msg.z:.3f})")
        if self.left_arm is None:
            rospy.logerr("Left MoveGroup not initialized!")
            return

        self.left_arm.set_position_target([point_msg.x, point_msg.y, point_msg.z])
        success, plan, _, _ = self.left_arm.plan()
        if success:
            rospy.loginfo("[Left] Planning succeeded, executing...")
            self.execute_cb_left(plan)
        else:
            rospy.logwarn("[Left] Planning failed!")

    def move_to_right_position_callback(self, point_msg):
        rospy.loginfo(f"[Right] Received target position: ({point_msg.x:.3f}, {point_msg.y:.3f}, {point_msg.z:.3f})")
        if self.right_arm is None:
            rospy.logerr("Right MoveGroup not initialized!")
            return

        self.right_arm.set_position_target([point_msg.x, point_msg.y, point_msg.z])
        success, plan, _, _ = self.right_arm.plan()
        if success:
            rospy.loginfo("[Right] Planning succeeded, executing...")
            self.execute_cb_right(plan)
        else:
            rospy.logwarn("[Right] Planning failed!")

    def execute_cb_left(self, plan):
        traj = plan.joint_trajectory
        left_traj = JointTrajectory()
        left_traj.joint_names = self.left_joint_names
        left_traj.header = traj.header
        left_traj.points = traj.points

        goal = FollowJointTrajectoryGoal(trajectory=left_traj)
        self.left_client.send_goal(goal)
        if self.left_client.wait_for_result(rospy.Duration(10.0)):
            rospy.loginfo("[Left] Execution completed.")
        else:
            rospy.logerr("[Left] Execution timed out!")


    def execute_cb_right(self, plan):
        traj = plan.joint_trajectory
        right_traj = JointTrajectory()
        right_traj.joint_names = self.right_joint_names
        right_traj.header = traj.header
        right_traj.points = traj.points

        goal = FollowJointTrajectoryGoal(trajectory=right_traj)
        self.right_client.send_goal(goal)
        if self.right_client.wait_for_result(rospy.Duration(10.0)):
            rospy.loginfo("[Right] Execution completed.")
        else:
            rospy.logerr("[Right] Execution timed out!")


if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    listener = TrajectoryListener()
    rospy.spin()


#  position: x: 0.18554691241174123
# y: 0.6508358316846692
# z: 1.0292827070371384