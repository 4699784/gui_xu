#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray, Pose

#################################
###path轨迹规划
#################################
def publish_multi_point_path():
    pub = rospy.Publisher('/right/target_cartesian_path', PoseArray, queue_size=1, latch=True)
    
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()

    base_pose = Pose()
    base_pose.position.x = 0.17418035221953315
    base_pose.position.y = 0.5583811855541566
    base_pose.position.z = 0.7702718536349595
    base_pose.orientation.x = 0.6583512290228323
    base_pose.orientation.y = 0.6586083874700932
    base_pose.orientation.z = -0.2573860849786028
    base_pose.orientation.w = 0.2579942915212624

    # 定义 4 个偏移点（例如：方形路径的四个角）
    offsets = [
        (0.0, 0.0),    # 起点
        (0.02, 0.0),   # 向右
        (0.02, 0.02),  # 向前
        (0.0, 0.02),   # 向左
        (0.0, 0.0)   # 可选：回到起点，形成闭合路径
    ]

    for dx, dy in offsets:
        p = Pose()
        p.position.x = base_pose.position.x + dx
        p.position.y = base_pose.position.y + dy
        p.position.z = base_pose.position.z
        p.orientation = base_pose.orientation
        pose_array.poses.append(p)

    pub.publish(pose_array)
    rospy.loginfo(f"Published {len(pose_array.poses)} waypoints")

if __name__ == '__main__':
    rospy.init_node('multi_point_publisher', anonymous=True)
    publish_multi_point_path()
    rospy.sleep(10.0)  