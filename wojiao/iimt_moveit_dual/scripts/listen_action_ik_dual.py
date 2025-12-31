#!/usr/bin/env python3
import rospy
import actionlib
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Point,Pose
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from hand_control import send_left_hand,send_right_hand
from reset_arms import reset_arms

################################################
##待完善的点：时序问题，阻塞问题
##单点的笛卡尔空间规划
################################################

class TrajectoryListener:
    def __init__(self):
        moveit_commander.roscpp_initialize([])

        self.default_euler = (0.0, 1.57, 0.0)

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
        self.left_sub = rospy.Subscriber('/left/target_pose', Pose, self.move_to_left_pose_callback)
        self.right_sub = rospy.Subscriber('/right/target_pose', Pose, self.move_to_right_pose_callback)

        #手臂时序发布者
        self.left_done_pub = rospy.Publisher('/left/arm_done', Bool, queue_size=1)
        self.right_done_pub = rospy.Publisher('/right/arm_done', Bool, queue_size=1)

        rospy.loginfo("Trajectory listener started. Waiting for /left/target_pose and /right/target_pose ...")
    def move_to_left_pose_callback(self, pose_msg):
        rospy.loginfo(f"[Left] Received target pose: "
                    f"pos=({pose_msg.position.x:.3f}, {pose_msg.position.y:.3f}, {pose_msg.position.z:.3f}), "
                    f"ori=({pose_msg.orientation.x:.3f}, {pose_msg.orientation.y:.3f}, "
                    f"{pose_msg.orientation.z:.3f}, {pose_msg.orientation.w:.3f})")
        
        if self.left_arm is None:
            rospy.logerr("Left MoveGroup not initialized!")
            return

        # 直接使用接收到的 pose_msg（包含位置和朝向）
        waypoints = [pose_msg]  # 注意：这里已经是 Pose 类型
        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,
            0.01,   # 步长（米）
            False   # 不允许跳跃
        )

        if fraction > 0.95:
            rospy.loginfo("[Left] Cartesian path planned successfully.")
            self.execute_cb_left(plan)  # 如果要执行就取消注释
        else:
            rospy.logwarn(f"[Left] Cartesian path only {fraction*100:.1f}% planned!")
    def move_to_right_pose_callback(self, pose_msg):
        rospy.loginfo(f"[Right] Received target pose: "
                    f"pos=({pose_msg.position.x:.3f}, {pose_msg.position.y:.3f}, {pose_msg.position.z:.3f}), "
                    f"ori=({pose_msg.orientation.x:.3f}, {pose_msg.orientation.y:.3f}, "
                    f"{pose_msg.orientation.z:.3f}, {pose_msg.orientation.w:.3f})")
        
        if self.right_arm is None:
            rospy.logerr("Right MoveGroup not initialized!")
            return

        target_pose = Pose()
        target_pose.position = pose_msg.position
        target_pose.orientation = pose_msg.orientation

        waypoints = [target_pose]
        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,
            0.01,
            False
        )

        if fraction > 0.95:
            rospy.loginfo("[Right] Cartesian path planned successfully.")
            self.execute_cb_right(plan)
        else:
            rospy.logwarn(f"[Right] Cartesian path only {fraction*100:.1f}% planned!")
    def add_time_parametrization(self, trajectory, velocity=0.2):
        """
        给 JointTrajectory 添加线性时间戳
        :param velocity: 关节平均速度（rad/s），越小越慢越稳
        """
        if not trajectory.points:
            return

        # 计算总关节移动量
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
    def execute_cb_left(self, plan):
        traj = plan.joint_trajectory

        left_traj = JointTrajectory()
        left_traj.joint_names = self.left_joint_names
        left_traj.header = traj.header
        # left_traj.points = traj.points
        for pt in traj.points:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = [-p for p in pt.positions]
            new_pt.velocities = [-v for v in pt.velocities] if pt.velocities else []
            new_pt.accelerations = [-a for a in pt.accelerations] if pt.accelerations else []
            new_pt.time_from_start = pt.time_from_start
            left_traj.points.append(new_pt)
        
        #添加时间戳
        self.add_time_parametrization(left_traj, velocity=0.15)
        
        goal = FollowJointTrajectoryGoal(trajectory=left_traj)
        self.left_client.send_goal(goal)
        if self.left_client.wait_for_result(rospy.Duration(80.0)):
            send_left_hand([1000, 1000, 1000, 993, 1, 113])
            rospy.loginfo("[Left] Execution completed.")
            # self.left_done_pub.publish(Bool(data=True))
            # reset_arms()
        else:
            rospy.logerr("[Left] Execution timed out!")
    def execute_cb_right(self, plan):   
        traj = plan.joint_trajectory

        #添加时间戳
        self.add_time_parametrization(traj, velocity=0.15)

        right_traj = JointTrajectory()
        right_traj.joint_names = self.right_joint_names
        right_traj.header = traj.header
        right_traj.points = traj.points

        goal = FollowJointTrajectoryGoal(trajectory=right_traj)
        self.right_client.send_goal(goal)
        if self.right_client.wait_for_result(rospy.Duration(30.0)):
            rospy.loginfo("[Right] Execution completed.")
        else:
            rospy.logerr("[Right] Execution timed out!")

if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    listener = TrajectoryListener()
    rospy.spin()