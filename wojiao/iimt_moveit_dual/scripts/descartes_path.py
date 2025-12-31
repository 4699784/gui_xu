#!/usr/bin/env python3
import rospy
import actionlib
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
from hand_control import send_left_hand, send_right_hand
from reset_arms import reset_arms


class TrajectoryListener:
    def __init__(self):
        moveit_commander.roscpp_initialize([])

        self.right_joint_names = [
            "Joint_ul_r_shoulder_pitch",
            "Joint_ul_r_shoulder_roll",
            "Joint_ul_r_shoulder_yaw", 
            "Joint_ul_r_elbow_pitch",
            "Joint_ul_r_wrist_yaw",
            "Joint_ul_r_wrist_pitch"
        ]

        try:
            self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
            self.right_arm.set_planning_time(10.0)
            self.right_arm.set_goal_position_tolerance(0.1)
        except Exception as e:
            rospy.logerr(f"Failed to initialize MoveGroupCommander: {e}")
            self.right_arm = None

        # Action client for right arm
        self.right_client = actionlib.SimpleActionClient(
            '/right_arm_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        self.right_client.wait_for_server()
        rospy.loginfo("Connected to right arm action server.")

        # 订阅多点路径（PoseArray）
        self.right_cartesian_sub = rospy.Subscriber(
            '/right/target_cartesian_path',
            PoseArray,
            self.move_right_cartesian_path_callback
        )

        # 完成信号发布器（可选，当前未启用）
        self.right_done_pub = rospy.Publisher('/right/arm_done', Bool, queue_size=1)

        rospy.loginfo("Trajectory listener (multi-point only) started. Waiting for /right/target_cartesian_path...")

    def move_right_cartesian_path_callback(self, pose_array_msg):
        rospy.loginfo(f"[Right] Received Cartesian path with {len(pose_array_msg.poses)} waypoints")

        waypoints = pose_array_msg.poses
        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,
            0.01,   # 步长（米）
            False   # 不允许跳跃
        )

        if fraction > 0.95:
            rospy.loginfo(f"[Right] Multi-point path planned successfully ({fraction*100:.1f}%).")
            self.execute_cb_right(plan)
        else:
            rospy.logwarn(f"[Right] Only {fraction*100:.1f}% of path planned!")

    def add_time_parametrization(self, trajectory, velocity=0.15):
        """给 JointTrajectory 添加线性时间戳"""
        if not trajectory.points:
            return

        total_distance = 0.0
        prev_positions = trajectory.points[0].positions
        for point in trajectory.points[1:]:
            for i in range(len(prev_positions)):
                total_distance += abs(point.positions[i] - prev_positions[i])
            prev_positions = point.positions

        total_time = total_distance / velocity if velocity > 0 else 1.0
        dt = total_time / len(trajectory.points)

        for i, point in enumerate(trajectory.points):
            point.time_from_start = rospy.Duration(i * dt)

    def execute_cb_right(self, plan):   
        traj = plan.joint_trajectory
        self.add_time_parametrization(traj, velocity=0.15)

        right_traj = JointTrajectory()
        right_traj.joint_names = self.right_joint_names
        right_traj.header = traj.header
        right_traj.points = traj.points

        goal = FollowJointTrajectoryGoal(trajectory=right_traj)
        self.right_client.send_goal(goal)
        if self.right_client.wait_for_result(rospy.Duration(60.0)):
            rospy.loginfo("[Right] Execution completed.")
            # self.right_done_pub.publish(Bool(data=True))  # 如需完成信号可取消注释
        else:
            rospy.logerr("[Right] Execution timed out!")


if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    listener = TrajectoryListener()
    rospy.spin()