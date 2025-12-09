#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import copy
import threading
import time

from robot_control import LeftArmController, RightArmController  

class DualArmTrajectoryExecutor:
    def __init__(self):
        self.left_arm = LeftArmController()
        self.right_arm = RightArmController()

        self.dual_joint_names = self.right_arm.get_joint_names() + self.left_arm.get_joint_names()

        #listen in execute_trajectory
        self.server = actionlib.SimpleActionServer(
            "/execute_trajectory",
            ExecuteTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()

    def execute_cb(self, goal):
        trajectory = goal.trajectory.joint_trajectory

        if trajectory.joint_names != self.dual_joint_names:
            self.server.set_aborted(text="Joint names mismatch")
            return

        try:
            left_names = self.left_arm.get_joint_names()
            right_names = self.right_arm.get_joint_names()

            left_indices = [trajectory.joint_names.index(j) for j in left_names]
            right_indices = [trajectory.joint_names.index(j) for j in right_names]

            rate = rospy.Rate(50)  
            for point in trajectory.points:
                if self.server.is_preempt_requested():
                    self.server.set_preempted()
                    return

                positions = point.positions
                left_pos = [positions[i] for i in left_indices]
                right_pos = [positions[i] for i in right_indices]

                self.left_arm.send_cmd(left_pos)
                #self.right_arm.send_cmd(right_pos)

                rospy.sleep(point.time_from_start)

            self.server.set_succeeded()

        except Exception as e:
            self.server.set_aborted(text=str(e))

if __name__ == '__main__':
    rospy.init_node('dual_arm_executor')
    executor = DualArmTrajectoryExecutor()
    rospy.spin()