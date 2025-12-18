#!/usr/bin/env python3
# torso_trajectory_server.py
import sys, os, time
sys.path.insert(0, os.path.expanduser('~/Documents/IIMT-SDK/python/sdk_demo_py'))

import rospy, actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from zmq_client import ZMQClient, TorsoCmdMsg

JOINTS = ["Joint_DownLimb_Pitch_1","Joint_DownLimb_Pitch_2","Joint_DownLimb_Pitch_3","Joint_DownLimb_Yaw_1"]

class TorsoAction:
    def __init__(self):
        ns = "/torso_controller/follow_joint_trajectory"
        self.srv = actionlib.SimpleActionServer(ns, FollowJointTrajectoryAction,
                                                execute_cb=self.execute, auto_start=False)
        ep = rospy.get_param("~endpoint", "tcp://192.168.8.201:5600")
        self.cli = ZMQClient(ep)
        try: self.cli.send_request(TorsoCmdMsg(cmd='init'))
        except Exception as e: rospy.logwarn("Torso init failed: %s", e)
        self.srv.start()
        rospy.loginfo("Torso action server up at %s", ns)

    def execute(self, goal):
        res = FollowJointTrajectoryResult()
        fb  = FollowJointTrajectoryFeedback()
        # map incoming joint order to JOINTS
        idx = {n:i for i,n in enumerate(goal.trajectory.joint_names)}
        if not all(j in idx for j in JOINTS):
            res.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            self.srv.set_aborted(res, "Required joints missing")
            return

        start = rospy.Time.now()
        for pt in goal.trajectory.points:
            if self.srv.is_preempt_requested() or rospy.is_shutdown():
                res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                self.srv.set_preempted(res, "Preempted")
                return

            # sleep until this point’s time
            tgt_time = start + pt.time_from_start
            while rospy.Time.now() < tgt_time and not rospy.is_shutdown():
                rospy.sleep(0.002)

            # build positions in canonical order
            psrc = pt.positions if pt.positions else [0.0]*len(goal.trajectory.joint_names)
            # cmd = [float(psrc[idx[j]]) for j in JOINTS]
            joint_vals = [float(psrc[idx[j]]) for j in JOINTS]  # 4 values
            cmd = joint_vals + [0.0, 0.0]  # now 6 values
            try:
                self.cli.send_request(TorsoCmdMsg(cmd='servoj', data=cmd))
            except Exception as e:
                res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                self.srv.set_aborted(res, "ZMQ send failed: %s" % e)
                return

            fb.joint_names = list(JOINTS)
            fb.desired = JointTrajectoryPoint(positions=cmd, time_from_start=pt.time_from_start)
            self.srv.publish_feedback(fb)

        res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.srv.set_succeeded(res, "Done")

if __name__ == "__main__":
    rospy.init_node("torso_trajectory_server")
    TorsoAction()
    rospy.spin()
