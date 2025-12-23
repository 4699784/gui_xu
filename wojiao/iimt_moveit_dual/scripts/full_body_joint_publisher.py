#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from angles import normalize_angle

# 假设 zmq_client.py 在同级目录
from zmq_client import ArmCmdMsg, TorsoCmdMsg, HeadCmdMsg, ZMQClient


# -------------------- 底层读取类 --------------------
class ArmReader:
    """统一读取左/右臂 6-DOF"""
    def __init__(self, side: str, port: int):
        self.client = ZMQClient(f"tcp://192.168.8.201:{port}")
        self.joint_names = [
            f"Joint_{side}_shoulder_pitch",
            f"Joint_{side}_shoulder_roll",
            f"Joint_{side}_shoulder_yaw",
            f"Joint_{side}_elbow_pitch",
            f"Joint_{side}_wrist_yaw",
            f"Joint_{side}_wrist_pitch",
        ]
        self.client.send_request(ArmCmdMsg(cmd='init'))

    def get_pos(self):
        data = self.client.send_request(ArmCmdMsg(cmd='get_pos'))
        return [float(v) for v in data[:6]] if data else [0.0] * 6


# class TorsoReader:
#     """读取腰部 4-DOF：Pitch1, Pitch2, Pitch3, Yaw"""
#     def __init__(self, port: int = 5600):
#         self.client = ZMQClient(f"tcp://192.168.8.201:{port}")
#         self.joint_names = [
#             "Joint_DownLimb_Pitch_1",
#             "Joint_DownLimb_Pitch_2",
#             "Joint_DownLimb_Pitch_3",
#             "Joint_DownLimb_Yaw_1",
#         ]
#         self.client.send_request(TorsoCmdMsg(cmd='init'))

#     def get_pos(self):
#         data = self.client.send_request(TorsoCmdMsg(cmd='get_pos'))
#         return [float(v) for v in data[:4]] if data else [0.0] * 4

##绕开硬件的torso初始位置的校准
class TorsoReader:
    def __init__(self, port: int = 5600):
        self.client = ZMQClient(f"tcp://192.168.8.201:{port}")
        self.joint_names = [
            "Joint_DownLimb_Pitch_1",
            "Joint_DownLimb_Pitch_2",
            "Joint_DownLimb_Pitch_3",
            "Joint_DownLimb_Yaw_1",
        ]
        self.client.send_request(TorsoCmdMsg(cmd='init'))
        rospy.logwarn("TorsoReader: FORCING 'stood_up' pose for all reads!")
    def get_pos(self):
        #完全站立
        data = self.client.send_request(TorsoCmdMsg(cmd='get_pos'))
        rospy.loginfo(f"TorsoReader raw data: {data}")
        return [-1.2404, 2.0226, -0.704, 0.0]

class HeadReader:
    """读取头部 2-DOF：Pitch,  Yaw"""
    def __init__(self, port: int = 5587):
        self.client = ZMQClient(f"tcp://192.168.8.201:{port}")
        self.joint_names = [
            "Joint_Neck_Pitch_1",
            "Joint_Neck_Yaw_1",
        ]
        self.client.send_request(HeadCmdMsg(cmd='init'))

    def get_pos(self):
        data = self.client.send_request(HeadCmdMsg(cmd='get_pos'))
        return [float(v) for v in data[:2]] if data else [0.0] * 2
    
class WheelReader:
    """读取头部 2-DOF：Pitch,  Yaw"""
    def __init__(self, port: int = 5587):
        self.joint_names = [
            "Joint_Wheel_T_RF",
            "Joint_Wheel_M_RF",
            "Joint_Wheel_T_LF",
            "Joint_Wheel_M_LF",
            "Joint_Wheel_T_RB",
            "Joint_Wheel_M_RB",
            "Joint_Wheel_T_LB",
            "Joint_Wheel_M_LB",
        ]

    def get_pos(self):

        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



# -------------------- ROS 发布器 --------------------
class FullBodyPublisher:
    def __init__(self):
        # 三个读取器
        self.left  = ArmReader("ul_l", 5589)
        self.right = ArmReader("ul_r", 5588)#rev
        self.torso = TorsoReader(5600)
        self.head = HeadReader(5587)
        self.wheels = WheelReader()
        # 发布器
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.tf_br = TransformBroadcaster()
        rospy.init_node('full_body_joint_publisher')
        self.rate = rospy.Rate(50)

    def spin(self):
        while not rospy.is_shutdown():
            js = JointState()
            js.header.stamp = rospy.Time.now()

            # 1. 左臂（Left arm）
            l_pos = self.left.get_pos()
            # Normalize each joint angle to (-pi, pi]
            l_pos = [normalize_angle(p) for p in l_pos]
            js.name.extend(self.left.joint_names)
            js.position.extend(l_pos)

            # 2. 右臂（Right arm）
            r_pos = self.right.get_pos()
            r_pos = [normalize_angle(p) for p in r_pos]
            js.name.extend(self.right.joint_names)
            js.position.extend(r_pos)

            # 3. 腰部
            t_pos = self.torso.get_pos()
            js.name.extend(self.torso.joint_names)
            js.position.extend(t_pos)
            # 4. 头部
            h_pos = self.head.get_pos()
            js.name.extend(self.head.joint_names)
            js.position.extend(h_pos)

            #5. 轮子
            w_pos = self.wheels.get_pos()
            js.name.extend(self.wheels.joint_names)
            js.position.extend(w_pos)

            self.pub.publish(js)

            # 可选：发布一个 world -> dummy_link 的静态 TF
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id  = "dummy_link"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_br.sendTransform(t)

            self.rate.sleep()


# -------------------- 入口 --------------------
if __name__ == '__main__':
    try:
        FullBodyPublisher().spin()
    except rospy.ROSInterruptException:
        pass