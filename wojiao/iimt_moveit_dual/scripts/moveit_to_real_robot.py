#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, actionlib, time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from robot_control import RightArmController
from loguru import logger

# Try to import a dedicated left controller; if missing, we’ll reuse right.
try:
    from robot_control import LeftArmController
except Exception:
    LeftArmController = None

LEFT_ENDPOINT_GUESS = "tcp://192.168.8.201:5589"  # only used if we reuse RightArmController
# Try to import a dedicated head controller; if missing, reuse RightArmController.
try:
    from robot_control import HeadController
except Exception:
    HeadController = None

HEAD_ENDPOINT_GUESS = "tcp://192.168.8.201:5587"  # head port from your mapping


class Bridge:
    def __init__(self):
        # ABSOLUTE COMPAT: same node name as before
        rospy.init_node('moveit_to_real_robot')

        # ----- RIGHT ARM (unchanged) -----
        self.right_arm = RightArmController()
        self.right_server = actionlib.SimpleActionServer(
            '/right_arm_controller/follow_joint_trajectory',   # ABSOLUTE COMPAT: same action name
            FollowJointTrajectoryAction,
            self.execute_right,
            False
        )
        self.right_server.start()
        rospy.loginfo("Right bridge ready.")

        # ----- LEFT ARM (new) -----
        # Prefer a dedicated LeftArmController; otherwise reuse RightArmController and retarget endpoint if possible.
        if LeftArmController is not None:
            self.left_arm = LeftArmController()
        else:
            rospy.logwarn("LeftArmController not available; reusing RightArmController for left arm.")
            self.left_arm = RightArmController()
            try:
                if hasattr(self.left_arm, "client") and hasattr(self.left_arm.client, "endpoint"):
                    self.left_arm.client.endpoint = LEFT_ENDPOINT_GUESS
                    rospy.logwarn("Retargeted left controller endpoint to %s", LEFT_ENDPOINT_GUESS)
            except Exception as e:
                rospy.logwarn("Could not retarget left controller endpoint: %s", e)

        self.left_server = actionlib.SimpleActionServer(
            '/left_arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            self.execute_left,
            False
        )
        self.left_server.start()
        rospy.loginfo("Left bridge ready.")
        
        # ----- HEAD (new) -----
        if HeadController is not None:
            self.head = HeadController()
        else:
            rospy.logwarn("HeadController not available; reusing RightArmController for head.")
            self.head = RightArmController()
            try:
                if hasattr(self.head, "client") and hasattr(self.head.client, "endpoint"):
                    self.head.client.endpoint = HEAD_ENDPOINT_GUESS
                    rospy.logwarn("Retargeted head controller endpoint to %s", HEAD_ENDPOINT_GUESS)
            except Exception as e:
                rospy.logwarn("Could not retarget head controller endpoint: %s", e)

        self.head_server = actionlib.SimpleActionServer(
            '/head_controller/follow_joint_trajectory',  # ABSOLUTE COMPAT
            FollowJointTrajectoryAction,
            self.execute_head,
            False
        )
        self.head_server.start()
        rospy.loginfo("Head bridge ready.")


    # ------- helpers to use MoveIt's timing/speed and cap at 0.5 -------
    @staticmethod
    def _segment_dt(prev_t, curr_t):
        dt = (curr_t - prev_t)
        # guard rails: tiny positive dt to avoid divide-by-zero, no negative sleeps
        return max(dt, 1e-3)

    @staticmethod
    def _infer_speed_ratio(joint_names, pt, prev_positions, dt):
        """
        Returns a scalar speed ratio in [0, 0.5] using:
         - MoveIt's per-point velocities if present,
         - otherwise an estimate from position delta / dt.
        Assumes the controller treats 1.0 as 'full speed'; we cap at 0.5.
        """
        # Prefer explicit velocities from MoveIt if provided
        if getattr(pt, "velocities", None) and len(pt.velocities) == len(joint_names):
            vmax = max(abs(v) for v in pt.velocities) if pt.velocities else 0.0
        else:
            # Estimate from finite difference if velocities are absent
            vmax = 0.0
            if prev_positions is not None and pt.positions and len(pt.positions) == len(joint_names):
                for curr, prev in zip(pt.positions, prev_positions):
                    vmax = max(vmax, abs((curr - prev) / dt))
        # Map vmax -> controller ratio. If your controller expects raw rad/s, this
        # still keeps things safe by capping hard at 0.5; small vmax gives small ratio.
        ratio = min(0.5, max(0.0, float(vmax)))
        return ratio

    # ====== RIGHT: ABSOLUTE COMPATIBLE EXECUTE ======
    def execute_right(self, goal):
        print('11111111111111111111111111')  # keep your print
        correct_joint_names = [
            'Joint_ul_r_shoulder_pitch', 'Joint_ul_r_shoulder_roll', 'Joint_ul_r_shoulder_yaw',
            'Joint_ul_r_elbow_pitch',    'Joint_ul_r_wrist_yaw',     'Joint_ul_r_wrist_pitch'
        ]
        joint_name_map = {name: i for i, name in enumerate(goal.trajectory.joint_names)}

        prev_t = 0.0
        prev_positions = None

        for pt in goal.trajectory.points:
            # reorder positions
            positions = list(pt.positions)
            reordered_positions = [positions[joint_name_map[name]] for name in correct_joint_names]

            # derive timing from MoveIt (use time_from_start to preserve planned tempo)
            curr_t = float(pt.time_from_start.to_sec()) if hasattr(pt.time_from_start, 'to_sec') else float(pt.time_from_start)
            dt = self._segment_dt(prev_t, curr_t)

            # infer a per-point speed ratio from MoveIt's velocities/tempo, capped at 0.5
            speed_ratio = self._infer_speed_ratio(goal.trajectory.joint_names, pt, prev_positions, dt)

            data = reordered_positions + [speed_ratio]  # <-- replaces fixed 0.05 with capped MoveIt-derived ratio
            logger.info(f"{data}")
            self.right_arm.send_cmd(data)

            # sleep according to MoveIt's timing (not fixed 0.1s)
            time.sleep(dt)

            prev_t = curr_t
            prev_positions = positions

        self.right_server.set_succeeded(FollowJointTrajectoryResult())
        rospy.loginfo("Right trajectory execution complete.")

    # ====== LEFT: mirrored logic for left joints (same timing & capped speed) ======
    def execute_left(self, goal):
        correct_joint_names = [
            'Joint_ul_l_shoulder_pitch', 'Joint_ul_l_shoulder_roll', 'Joint_ul_l_shoulder_yaw',
            'Joint_ul_l_elbow_pitch',    'Joint_ul_l_wrist_yaw',     'Joint_ul_l_wrist_pitch'
        ]
        joint_name_map = {name: i for i, name in enumerate(goal.trajectory.joint_names)}

        prev_t = 0.0
        prev_positions = None

        for pt in goal.trajectory.points:
            positions = list(pt.positions)
            reordered_positions = [positions[joint_name_map[name]] for name in correct_joint_names]

            curr_t = float(pt.time_from_start.to_sec()) if hasattr(pt.time_from_start, 'to_sec') else float(pt.time_from_start)
            dt = self._segment_dt(prev_t, curr_t)

            speed_ratio = self._infer_speed_ratio(goal.trajectory.joint_names, pt, prev_positions, dt)

            data = reordered_positions + [speed_ratio]  # keep same tail semantics, now dynamic & capped at 0.5
            logger.info(f"{data}")
            self.left_arm.send_cmd(data)

            time.sleep(dt)

            prev_t = curr_t
            prev_positions = positions

        self.left_server.set_succeeded(FollowJointTrajectoryResult())
        rospy.loginfo("Left trajectory execution complete.")

    # ====== HEAD: neck yaw/pitch (same timing & capped speed) ======
    def execute_head(self, goal):
        # Match these to your URDF exactly (swap order if your hardware expects Pitch, Yaw)
        correct_joint_names = [
            'Joint_Neck_Yaw_1',
            'Joint_Neck_Pitch_1'
        ]
        name_to_idx = {n: i for i, n in enumerate(goal.trajectory.joint_names)}

        prev_t = 0.0
        prev_positions = None

        try:
            for pt in goal.trajectory.points:
                positions = list(pt.positions)

                # Reorder to device order (two joints only)
                try:
                    yaw  = positions[name_to_idx[correct_joint_names[0]]]
                    pitch = positions[name_to_idx[correct_joint_names[1]]]
                except KeyError as e:
                    rospy.logerr(f"[head] Missing joint in goal: {e}. Goal has: {goal.trajectory.joint_names}")
                    self.head_server.set_aborted(FollowJointTrajectoryResult())
                    return

                # Preserve MoveIt's tempo (no fixed sleeps)
                curr_t = float(pt.time_from_start.to_sec()) if hasattr(pt.time_from_start, 'to_sec') else float(pt.time_from_start)
                dt = self._segment_dt(prev_t, curr_t)

                # IMPORTANT: head driver likely expects exactly 2 angles (no speed tail)
                payload = [float(yaw), float(pitch)]
                logger.info(f"[head->send_cmd] {payload}")
                self.head.send_cmd(payload)   # <-- no speed appended

                time.sleep(dt)
                prev_t = curr_t
                prev_positions = positions

        except Exception as e:
            rospy.logerr(f"[head] Driver error: {e}")
            self.head_server.set_aborted(FollowJointTrajectoryResult())
            return

        self.head_server.set_succeeded(FollowJointTrajectoryResult())
        rospy.loginfo("Head trajectory execution complete.")


if __name__ == "__main__":
    Bridge()
    rospy.spin()
