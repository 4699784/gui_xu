#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from reset_arms import reset_arms
from hand_control import send_left_hand
def right_publish_target_and_reset(x, y, z, wait_duration=10):
    #发布目标位置到 /right/target_position，并在等待一段时间后归零机械臂。

    if not rospy.core.is_initialized():
        rospy.init_node('grasp', anonymous=True)

    pub = rospy.Publisher('/right/target_position', Point, queue_size=1,latch=True)

    point = Point()
    point.x = x
    point.y = y
    point.z = z

    pub.publish(point)
    rospy.loginfo("Published target position: x=%.3f, y=%.3f, z=%.3f", x, y, z)

    rospy.sleep(wait_duration)

def left_publish_target_and_reset(x, y, z, wait_duration=10):
    #发布目标位置到 /right/target_position，并在等待一段时间后归零机械臂。

    if not rospy.core.is_initialized():
        rospy.init_node('grasp', anonymous=True)

    pub = rospy.Publisher('/left/target_position', Point, queue_size=1,latch=True)

    point = Point()
    point.x = x
    point.y = y
    point.z = z

    pub.publish(point)
    rospy.loginfo("Published target position: x=%.3f, y=%.3f, z=%.3f", x, y, z)

    rospy.sleep(wait_duration)
    send_left_hand([1000, 1000, 1000, 993, 1, 113])
    rospy.sleep(wait_duration)

# ========================
# 主程序：抓取
# ========================
if __name__ == '__main__':
    try:
        #right_publish_target_and_reset(x=0.185, y=0.65, z=1.02, wait_duration=10)
        left_publish_target_and_reset(x=-0.23, y=-0.42, z=0.88, wait_duration=10)


        # 你也可以多次调用不同位置（取消注释即可）：
        # publish_target_and_reset(0.2, 0.5, 0.95, wait_duration=5)
        # publish_target_and_reset(-0.1, 0.6, 1.1, wait_duration=8)

        #回归零位
        reset_arms()
        send_left_hand([1000, 1000, 1000, 1000, 1000, 0])

    except rospy.ROSInterruptException:
        pass
