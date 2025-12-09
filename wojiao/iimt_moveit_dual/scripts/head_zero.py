import rospy
import moveit_commander

rospy.init_node("head_zero_pose_check")
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("head")  

# Get current state, then set all joints to 0
zero_pose = {joint: 0.0 for joint in group.get_active_joints()}
group.set_joint_value_target(zero_pose)
pose = group.get_current_pose().pose  # This reflects the end-effector at zero config
print("Head end-effector pose at zero config:", pose)