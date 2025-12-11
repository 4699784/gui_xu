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
            self.left_arm.set_goal_position_tolerance(0.01)
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

        # Action Server
        self.server = actionlib.SimpleActionServer(
            "/execute_trajectory",
            ExecuteTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        
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
        
        # 创建目标位姿
        target_pose = Pose()
        target_pose.position.x = point_msg.x
        target_pose.position.y = point_msg.y
        target_pose.position.z = point_msg.z
        target_pose.orientation = current_pose.orientation  # 保持当前方向
        
        # 设置目标位姿
        self.left_arm.set_pose_target(target_pose)
        
        # 规划并执行
        success, plan_trajectory, planning_time, error_code = self.left_arm.plan()
        if success and len(plan_trajectory.joint_trajectory.points) > 0:
            rospy.loginfo("Planning succeeded, executing...")
            exec_success = self.left_arm.execute(plan_trajectory, wait=True)
            self.left_arm.stop()
            self.left_arm.clear_pose_targets()
            
            if exec_success:
                rospy.loginfo("Successfully moved to target position")
            else:
                rospy.logerr("Failed to execute trajectory")
        else:
            rospy.logerr("Failed to plan trajectory to target position (success=%s, points=%d)",
                        success, len(plan_trajectory.joint_trajectory.points) if success else 0)

    #tarjectory execute
    def execute_cb(self, goal):
        traj = goal.trajectory.joint_trajectory

        # 分离左右臂轨迹
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
        
        # 等待执行完成
        left_done = self.left_client.wait_for_result(rospy.Duration(10.0)) 
        right_done = self.right_client.wait_for_result(rospy.Duration(10.0))

        self.server.set_succeeded()
        rospy.loginfo("Trajectory processing completed.")

if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    listener = TrajectoryListener()
    rospy.spin()