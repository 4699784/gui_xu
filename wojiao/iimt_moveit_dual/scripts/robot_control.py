#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ZMQ message classes
from zmq_client import ZMQClient, ArmCmdMsg, HandCmdMsg, TorsoCmdMsg, HeadCmdMsg

# =============================
# Config: endpoints & joint sets
# =============================
IP = "192.168.8.201" 

ENDPOINTS = {
    "right_arm": f"tcp://{IP}:5588",
    "left_arm":  f"tcp://{IP}:5589",
    "head":      f"tcp://{IP}:5587",
    "left_hand": f"tcp://{IP}:5590",
    "right_hand":f"tcp://{IP}:5591",
    "torso":     f"tcp://{IP}:5600",
}

JOINTS = {
    # --- Arms (from your ros_controllers.yaml)
    "right_arm": [
        "Joint_ul_r_shoulder_pitch",
        "Joint_ul_r_shoulder_roll",
        "Joint_ul_r_shoulder_yaw",
        "Joint_ul_r_elbow_pitch",
        "Joint_ul_r_wrist_yaw",
        "Joint_ul_r_wrist_pitch",
    ],
    "left_arm": [
        "Joint_ul_l_shoulder_pitch",
        "Joint_ul_l_shoulder_roll",
        "Joint_ul_l_shoulder_yaw",
        "Joint_ul_l_elbow_pitch",
        "Joint_ul_l_wrist_yaw",
        "Joint_ul_l_wrist_pitch",
    ],

    # --- Optional groups (names here follow your commented YAML)
    "head": [
        "Joint_Neck_Yaw_1",
        "Joint_Neck_Pitch_1",
    ],
    "left_hand": [
        "Joint_ul_l_hand",
    ],
    "right_hand": [
        "Joint_ul_r_hand",
    ],
    "torso": [
        "Joint_torso_yaw",
        "Joint_torso_pitch",
        "Joint_torso_roll",
    ],
}

MSG_CLASS = {
    "right_arm": ArmCmdMsg,
    "left_arm":  ArmCmdMsg,
    "head":      HeadCmdMsg,
    "left_hand": HandCmdMsg,
    "right_hand":HandCmdMsg,
    "torso":     TorsoCmdMsg,
}

# =============================
# Base class (keeps old API)
# =============================
class _BaseZMQController:
    """
    Backward-compatible shape:
      - get_pos(), send_cmd(data), publish_loop()
      - start_publishing(rate_hz), stop()
      - get_joint_names(), read_joint_positions/velocities/efforts()
    """
    def __init__(self, group_name, topic="/joint_states", tf_child="dummy_link"):
        self.group = group_name
        self.joint_names = list(JOINTS[group_name])
        self.endpoint = ENDPOINTS[group_name]
        self.msg_cls = MSG_CLASS[group_name]

        self.client = ZMQClient(self.endpoint)
        self.pub = rospy.Publisher(topic, JointState, queue_size=20)
        self.tf_broadcaster = TransformBroadcaster()

        self._pub_thread = None
        self._stop_evt = threading.Event()

        try:
            self.client.send_request(self.msg_cls(cmd='init'))
        except Exception as e:
            rospy.logwarn("[%s] init failed: %s", self.group, e)

        # static TF child name (can be customized per controller if you like)
        self._tf_child = tf_child

        # legacy loop default rate
        self.rate = rospy.Rate(10)

    # ---- Old API
    def get_pos(self):
        """Return current positions list or None."""
        try:
            data = self.client.send_request(self.msg_cls(cmd='get_pos'))
            if not data:
                return None
            return list(data[:len(self.joint_names)])
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[%s] get_pos error: %s", self.group, e)
            return None

    def send_cmd(self, data):
        """Send desired joint positions (list length must match this group)."""
        try:
            return self.client.send_request(self.msg_cls(cmd='servoj', data=list(data)))
        except Exception as e:
            rospy.logerr("[%s] send_cmd failed: %s", self.group, e)
            return None

    def publish_loop(self):
        """Legacy single-thread loop at 10 Hz."""
        while not rospy.is_shutdown():
            self._publish_once()
            self.rate.sleep()

    # ---- Compat helpers
    def start_publishing(self, rate_hz=50):
        """Spawn a background publisher thread at rate_hz."""
        if self._pub_thread and self._pub_thread.is_alive():
            return
        self._stop_evt.clear()
        self._pub_thread = threading.Thread(
            target=self._publish_worker, args=(rate_hz,), daemon=True
        )
        self._pub_thread.start()

    def stop(self):
        self._stop_evt.set()

    def _publish_worker(self, rate_hz):
        r = rospy.Rate(rate_hz)
        while not rospy.is_shutdown() and not self._stop_evt.is_set():
            self._publish_once()
            r.sleep()

    def _publish_once(self):
        pos = self.read_joint_positions()
        if pos is None:
            return
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self.joint_names
        js.position = pos

        vel = self.read_joint_velocities()
        eff = self.read_joint_efforts()
        if vel: js.velocity = vel
        if eff: js.effort = eff
        self.pub.publish(js)

        # Static TF (world -> self._tf_child)
        t = TransformStamped()
        t.header.stamp = js.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = self._tf_child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    # ---- Accessors that other layers expect
    def get_joint_names(self):
        return list(self.joint_names)

    def read_joint_positions(self):
        return self.get_pos()

    def read_joint_velocities(self):
        pos = self.get_pos()
        return [0.0] * len(pos) if pos is not None else None

    def read_joint_efforts(self):
        pos = self.get_pos()
        return [0.0] * len(pos) if pos is not None else None


# =============================
# Concrete controllers (same API)
# =============================
class RightArmController(_BaseZMQController):
    def __init__(self):
        super().__init__("right_arm", tf_child="r_arm_root")

class LeftArmController(_BaseZMQController):
    def __init__(self):
        super().__init__("left_arm", tf_child="l_arm_root")

class HeadController(_BaseZMQController):
    def __init__(self):
        super().__init__("head", tf_child="head_root")

class TorsoController(_BaseZMQController):
    def __init__(self):
        super().__init__("torso", tf_child="torso_root")

class LeftHandController(_BaseZMQController):
    def __init__(self):
        super().__init__("left_hand", tf_child="l_hand_root")

class RightHandController(_BaseZMQController):
    def __init__(self):
        super().__init__("right_hand", tf_child="r_hand_root")


if __name__ == "__main__":
    rospy.init_node('robot_controller_node')
    import sys
    name = sys.argv[1] if len(sys.argv) > 1 else "right_arm"
    cls = {
        "right_arm": RightArmController,
        "left_arm":  LeftArmController,
        "head":      HeadController,
        "torso":     TorsoController,
        "left_hand": LeftHandController,
        "right_hand":RightHandController,
    }.get(name, RightArmController)
    ctrl = cls()
    ctrl.publish_loop()
