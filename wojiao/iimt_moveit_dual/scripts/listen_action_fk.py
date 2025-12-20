#!/usr/bin/env python3

import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryAction,ExecuteTrajectoryGoal,ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import moveit_commander
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest

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

        # Action clients for real robot controllers
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

        # Action server to receive trajectories from MoveIt!
        self.server = actionlib.SimpleActionServer(
            "/execute_trajectory",
            ExecuteTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("Trajectory listener started. Waiting for MoveIt! trajectories...")
        self.goal_pub = rospy.Publisher('/recorded_moveit_trajectory',JointTrajectory,queue_size=10)

        # MoveIt! interfaces
        self.left_move_group = moveit_commander.MoveGroupCommander("left_arm")
        self.right_move_group = moveit_commander.MoveGroupCommander("right_arm")

        # FK service client
        try:
            rospy.wait_for_service('/compute_fk', timeout=10.0)
            self.fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)
            rospy.loginfo("Connected to /compute_fk service.")
        except rospy.ROSException as e:
            rospy.logerr("Failed to connect to /compute_fk service: %s", str(e))
            raise

        # End-effector links
        self.left_ee_link = self.left_move_group.get_end_effector_link() or self.left_move_group.get_name() + "_ee"
        self.right_ee_link = self.right_move_group.get_end_effector_link() or self.right_move_group.get_name() + "_ee"
        rospy.loginfo("Left EE link: %s", self.left_ee_link)
        rospy.loginfo("Right EE link: %s", self.right_ee_link)

    def compute_fk(self, joint_positions, is_left=True):
        try:
            group = self.left_move_group if is_left else self.right_move_group
            ee_link = self.left_ee_link if is_left else self.right_ee_link
            joint_names = self.left_joint_names if is_left else self.right_joint_names

            if len(joint_positions) != len(joint_names):
                rospy.logerr("FK input size mismatch: got %d, expected %d", len(joint_positions), len(joint_names))
                return None

            req = GetPositionFKRequest()
            req.header.frame_id = group.get_planning_frame()
            req.fk_link_names = [ee_link]
            req.robot_state.joint_state.name = joint_names
            req.robot_state.joint_state.position = joint_positions  # ← 注意：是 position（单数）

            resp = self.fk_service(req)

            if resp.error_code.val != 1:  # 1 == SUCCESS
                rospy.logerr("FK service failed with error code: %d", resp.error_code.val)
                return None

            if len(resp.pose_stamped) == 0:
                rospy.logerr("FK response has no pose.")
                return None

            return resp.pose_stamped[0].pose

        except rospy.ServiceException as e:
            rospy.logerr("FK service call failed: %s", str(e))
            return None

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
            # left_pt.positions = pt.positions[:6]
            # left_pt.velocities = pt.velocities[:6] if pt.velocities else []
            # left_pt.accelerations = pt.accelerations[:6] if pt.accelerations else []
            #原始的轨迹
            original_left_positions = pt.positions[:6]
            # 在分割轨迹后，对 left_pt.positions 取反
            inverted_left_positions = [-p for p in pt.positions[:6]]
            left_pt.positions = inverted_left_positions
            left_pt.velocities = [-v for v in pt.velocities[:6]] if pt.velocities else []
            left_pt.accelerations = [-a for a in pt.accelerations[:6]] if pt.accelerations else []
            left_pt.time_from_start = pt.time_from_start
            left_traj.points.append(left_pt)

            right_pt = JointTrajectoryPoint()
            right_pt.positions = pt.positions[6:]
            right_pt.velocities = pt.velocities[6:] if pt.velocities else []
            right_pt.accelerations = pt.accelerations[6:] if pt.accelerations else []
            right_pt.time_from_start = pt.time_from_start
            right_traj.points.append(right_pt)

            last_original_left = original_left_positions

        # Compute FK for the last point (optional: you can loop over all points if needed)
        left_pose = self.compute_fk(last_original_left, is_left=True)
        right_pose = self.compute_fk(right_pt.positions, is_left=False)

        if left_pose and right_pose:
            rospy.loginfo("Left EE pose:\n  position: %s\n  orientation: %s",
                  left_pose.position, left_pose.orientation)
            rospy.loginfo("Right EE pose:\n  position: %s\n  orientation: %s",
                  right_pose.position, right_pose.orientation)
        else:
            rospy.logwarn("Failed to compute FK for one or both arms.")

        # Send trajectories to real robot
        left_goal = FollowJointTrajectoryGoal(trajectory=left_traj)
        right_goal = FollowJointTrajectoryGoal(trajectory=right_traj)

        self.left_client.send_goal(left_goal)
        self.right_client.send_goal(right_goal)

        left_done = self.left_client.wait_for_result(rospy.Duration(10.0))
        right_done = self.right_client.wait_for_result(rospy.Duration(10.0))

        if not (left_done and right_done):
            rospy.logwarn("One or both arms did not finish within timeout.")

        self.server.set_succeeded()
        rospy.loginfo("Trajectory execution completed.")



if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    listener = TrajectoryListener()
    rospy.spin()

'''
#[INFO] [1765437451.778048]: Left EE pose:
  position: x: -0.26467314436804507
y: -0.44280720504844195
z: -0.19065174648128067
  orientation: x: -0.2903821540331724
y: -0.6620691611892915
z: 0.47643947304053885
w: 0.5003479378892429
[INFO] [1765437451.781682]: Right EE pose:
  position: x: 0.2244709911787207
y: -0.20788259829684919
z: 0.7538348721129164
  orientation: x: -0.6949340250214676
y: -0.6577141874692004
z: -0.16831750520250927
w: 0.23695562013085034
'''

'''
测试点位1：
 "x: -0.20
y: -0.06
z: 0.23" 

测试点位2：
x: -0.3876866366201206
y: -0.4791706489493061
z: -0.13332274184987458



'''

