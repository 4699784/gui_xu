#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from reset_arms import reset_arms
from hand_control import send_left_hand
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

################################################
##待完善的点：时序问题，阻塞问题
################################################

# 全局变量：记录机械臂是否完成
left_arm_done = False
def right_publish_target_and_reset(x, y, z, ox, oy, oz, ow, wait_duration=10):
    #发布目标位置到 /right/target_position，并在等待一段时间后归零机械臂。
    pub = rospy.Publisher('/right/target_pose', Pose, queue_size=1,latch=True)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow

    pub.publish(pose)
    rospy.loginfo(
        "Published right target pose: pos=(%.3f, %.3f, %.3f), ori=(%.3f, %.3f, %.3f, %.3f)",
        x, y, z, ox, oy, oz, ow
    )
def left_arm_done_callback(msg):
    global left_arm_done 
    if msg.data:  
        left_arm_done = True
        rospy.loginfo("[Main] Received left arm done signal.")
def left_publish_target_and_reset(x, y, z, ox, oy, oz, ow, wait_duration=10):
    """
    发布左手目标位姿，并控制灵巧手。
    """
    left_arm_done = False

    pub = rospy.Publisher('/left/target_pose', Pose, queue_size=1, latch=True)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow

    pub.publish(pose)
    rospy.loginfo(
        "Published left target pose: pos=(%.3f, %.3f, %.3f), ori=(%.3f, %.3f, %.3f, %.3f)",
        x, y, z, ox, oy, oz, ow
    )

    # 订阅完成信号
    rospy.Subscriber('/left/arm_done', Bool, left_arm_done_callback)


# ========================
# 主程序：抓取
# ========================
if __name__ == '__main__':
    try:   
        rospy.init_node('grasp', anonymous=True)
        #test
        #left_test（伸直）
        # left_publish_target_and_reset(
        #     x=-0.20416027956997784,
        #     y=0.4984886802889051,
        #     z=0.6795714053068135,
        #     ox=-0.2207094379210395,
        #     oy=-0.766471147711345,
        #     oz=0.3390662159838744,
        #     ow=0.4988420841478891,
        #     wait_duration=20
        # )
        #right_test（已测试，可以）
        right_publish_target_and_reset(
            x=0.17418035221953315,
            y=0.5583811855541566,
            z=0.7702718536349595,
            ox=0.6583512290228323,
            oy=0.6586083874700932,
            oz=-0.2573860849786028,
            ow=0.2579942915212624,
            wait_duration=20
        )
        rospy.sleep(20)
        send_left_hand([1000, 1000, 1000, 1000, 1000, 0])
        #左手抓取姿态
        # left_publish_target_and_reset(
        # x=-0.20921624624911417,
        # y=0.47190890594134116,
        # z=0.9137471010370019,
        # ox=-0.007774883016590715,
        # oy=-0.7870769383811327,
        # oz=-0.02234496757156511,
        # ow=0.6164009625940633,
        # wait_duration=40
        # )
       
        #回归零位
        #reset_arms()
        # send_left_hand([1000, 1000, 1000, 1000, 1000, 0])

    except rospy.ROSInterruptException:
        pass