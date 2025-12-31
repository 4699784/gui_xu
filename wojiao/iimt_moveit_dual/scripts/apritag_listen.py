#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
def apriltag_callback(msg):
    if not msg.detections:
        return

    rospy.loginfo("=== AprilTag Detected ===")
    for detection in msg.detections:
        tag_id = detection.id
        pose = detection.pose.pose.pose  # geometry_msgs/Pose

        # 打印位姿
        rospy.loginfo(
            "Tag ID: %d | Pos: (%.4f, %.4f, %.4f) | Ori: (%.4f, %.4f, %.4f, %.4f)",
            tag_id,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
def main():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber('/apriltag/tag_detections', AprilTagDetectionArray, apriltag_callback)
    rospy.loginfo("Subscribed to /apriltag/tag_detections. Waiting for detections...")
    rospy.spin()

if __name__ == '__main__':
    main()