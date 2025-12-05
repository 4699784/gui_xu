#!/usr/bin/env python3
"""
arm_teach_and_playback.py

机械臂示教与回放工具 - 记录关节位置并可回放至这些位置

功能:
- 记录当前关节位置并命名
- 保存记录到文件或ROS参数服务器
- 回放至已记录的位置
- 列出所有已保存的位置
- 删除指定位置

使用方式:
1. 启动节点:
   rosrun your_package arm_teach_and_playback.py

2. 通过服务调用功能:
   rosservice call /arm_teach/record_position "home"
   rosservice call /arm_teach/move_to_position "home"
   rosservice call /arm_teach/list_positions
   rosservice call /arm_teach/delete_position "home"

3. 也可以通过rqt_service_caller或自定义UI访问这些服务
"""

import sys
import os
import json
import rospy
import tf2_ros
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse, SetString, SetStringResponse
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_commander import (
    roscpp_initialize, roscpp_shutdown,
    MoveGroupCommander, RobotCommander
)

class ArmTeachAndPlayback:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("arm_teach_and_playback", anonymous=False)
        
        # ROS参数
        self.group_name = rospy.get_param("~move_group", "mesonexus_dexter")
        self.storage_file = rospy.get_param("~storage_file", os.path.expanduser("~/.arm_teach_positions.json"))
        self.confirm_before_move = rospy.get_param("~confirm_before_move", True)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # MoveIt初始化
        self.robot = RobotCommander()
        self.group = MoveGroupCommander(self.group_name)
        
        # 关节位置存储
        self.taught_positions = {}
        
        # 速度/加速度限制
        self.vel_scale = rospy.get_param("~vel_scale", 0.3)
        self.acc_scale = rospy.get_param("~acc_scale", 0.3)
        
        self.group.set_max_velocity_scaling_factor(self.vel_scale)
        self.group.set_max_acceleration_scaling_factor(self.acc_scale)
        
        # 加载已保存的位置
        self.load_saved_positions()
        
        # 创建服务
        self.record_srv = rospy.Service('~record_position', SetString, self.handle_record_position)
        self.move_to_srv = rospy.Service('~move_to_position', SetString, self.handle_move_to_position)
        self.list_srv = rospy.Service('~list_positions', Trigger, self.handle_list_positions)
        self.delete_srv = rospy.Service('~delete_position', SetString, self.handle_delete_position)
        self.get_current_pose_srv = rospy.Service('~get_current_pose', Trigger, self.handle_get_current_pose)
        
        # 简单的命令行接口话题
        self.command_sub = rospy.Subscriber('~command', String, self.handle_command)
        
        rospy.loginfo("ArmTeachAndPlayback initialized. Services available:")
        rospy.loginfo("  ~record_position (std_srvs/SetString)")
        rospy.loginfo("  ~move_to_position (std_srvs/SetString)")
        rospy.loginfo("  ~list_positions (std_srvs/Trigger)")
        rospy.loginfo("  ~delete_position (std_srvs/SetString)")
        rospy.loginfo("  ~get_current_pose (std_srvs/Trigger)")
        rospy.loginfo("  ~command (std_msgs/String) - commands: record:<name>, move:<name>, list, delete:<name>")
        rospy.loginfo("Loaded %d saved positions from %s", len(self.taught_positions), self.storage_file)
    
    def load_saved_positions(self):
        """从文件加载已保存的位置"""
        try:
            if os.path.exists(self.storage_file):
                with open(self.storage_file, 'r') as f:
                    self.taught_positions = json.load(f)
                rospy.loginfo("Loaded %d positions from %s", len(self.taught_positions), self.storage_file)
            else:
                rospy.loginfo("No saved positions file found at %s, starting fresh", self.storage_file)
        except Exception as e:
            rospy.logerr("Error loading saved positions: %s", str(e))
            self.taught_positions = {}
    
    def save_positions(self):
        """保存位置到文件"""
        try:
            directory = os.path.dirname(self.storage_file)
            if not os.path.exists(directory):
                os.makedirs(directory)
            
            with open(self.storage_file, 'w') as f:
                json.dump(self.taught_positions, f, indent=2)
            rospy.loginfo("Saved %d positions to %s", len(self.taught_positions), self.storage_file)
        except Exception as e:
            rospy.logerr("Error saving positions: %s", str(e))
    
    def handle_record_position(self, req):
        """记录当前位置"""
        try:
            # 获取当前关节位置
            joint_values = self.group.get_current_joint_values()
            joint_names = self.group.get_active_joints()
            
            # 保存位置 - 包含关节值、名称、时间戳和当前末端执行器位姿
            position_name = req.data.strip()
            if not position_name:
                return SetStringResponse(success=False, message="Position name cannot be empty")
            
            # 获取当前末端执行器位姿
            current_pose = self.group.get_current_pose()
            
            self.taught_positions[position_name] = {
                'joint_names': joint_names,
                'joint_values': [float(v) for v in joint_values],
                'timestamp': rospy.Time.now().to_sec(),
                'end_effector_pose': {
                    'position': {
                        'x': current_pose.pose.position.x,
                        'y': current_pose.pose.position.y,
                        'z': current_pose.pose.position.z
                    },
                    'orientation': {
                        'x': current_pose.pose.orientation.x,
                        'y': current_pose.pose.orientation.y,
                        'z': current_pose.pose.orientation.z,
                        'w': current_pose.pose.orientation.w
                    }
                }
            }
            
            # 保存到文件
            self.save_positions()
            
            rospy.loginfo(f"Recorded position '{position_name}': {joint_values}")
            return SetStringResponse(success=True, message=f"Position '{position_name}' recorded successfully")
        except Exception as e:
            error_msg = f"Failed to record position: {str(e)}"
            rospy.logerr(error_msg)
            return SetStringResponse(success=False, message=error_msg)
    
    def handle_move_to_position(self, req):
        """移动到已记录位置"""
        try:
            position_name = req.data.strip()
            if not position_name:
                return SetStringResponse(success=False, message="Position name cannot be empty")
            
            if position_name not in self.taught_positions:
                available = ", ".join(self.taught_positions.keys())
                return SetStringResponse(success=False, message=f"Position '{position_name}' not found. Available positions: {available}")
            
            # 获取目标关节值
            target_joints = self.taught_positions[position_name]['joint_values']
            rospy.loginfo(f"Moving to position '{position_name}': {target_joints}")
            
            # 确认移动（如果启用）
            if self.confirm_before_move:
                rospy.loginfo(f"Press Enter to move to position '{position_name}' or Ctrl+C to cancel...")
                try:
                    input()
                except (KeyboardInterrupt, EOFError):
                    return SetStringResponse(success=False, message="Move cancelled by user")
            
            # 规划并执行
            self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target(target_joints)
            
            plan = self.group.plan()
            if isinstance(plan, tuple):
                success = plan[0]
                trajectory = plan[1]
            else:
                success = True if plan.joint_trajectory.points else False
                trajectory = plan
            
            if not success or not trajectory.joint_trajectory.points:
                return SetStringResponse(success=False, message=f"Planning to position '{position_name}' failed")
            
            # 执行运动
            self.group.execute(trajectory, wait=True)
            self.group.stop()
            
            return SetStringResponse(success=True, message=f"Successfully moved to position '{position_name}'")
        except Exception as e:
            error_msg = f"Failed to move to position: {str(e)}"
            rospy.logerr(error_msg)
            return SetStringResponse(success=False, message=error_msg)
    
    def handle_list_positions(self, req):
        """列出所有已保存的位置"""
        if not self.taught_positions:
            message = "No positions recorded yet"
            rospy.loginfo(message)
            return TriggerResponse(success=True, message=message)
        
        # 格式化输出
        output = []
        output.append(f"Total positions: {len(self.taught_positions)}")
        output.append("Available positions:")
        
        for name, data in self.taught_positions.items():
            timestamp = rospy.Time.from_sec(data['timestamp']).to_sec()
            joint_values = [f"{v:.3f}" for v in data['joint_values']]
            output.append(f"  - {name}: [{', '.join(joint_values)}] (recorded at {timestamp:.1f}s)")
        
        message = "\n".join(output)
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)
    
    def handle_delete_position(self, req):
        """删除指定位置"""
        position_name = req.data.strip()
        if not position_name:
            return SetStringResponse(success=False, message="Position name cannot be empty")
        
        if position_name not in self.taught_positions:
            return SetStringResponse(success=False, message=f"Position '{position_name}' not found")
        
        del self.taught_positions[position_name]
        self.save_positions()
        
        message = f"Position '{position_name}' deleted successfully"
        rospy.loginfo(message)
        return SetStringResponse(success=True, message=message)
    
    def handle_get_current_pose(self, req):
        """获取当前末端执行器位姿"""
        try:
            current_pose = self.group.get_current_pose()
            message = (
                f"Current end-effector pose in {current_pose.header.frame_id}:\n"
                f"Position: x={current_pose.pose.position.x:.4f}, y={current_pose.pose.position.y:.4f}, z={current_pose.pose.position.z:.4f}\n"
                f"Orientation: x={current_pose.pose.orientation.x:.4f}, y={current_pose.pose.orientation.y:.4f}, "
                f"z={current_pose.pose.orientation.z:.4f}, w={current_pose.pose.orientation.w:.4f}"
            )
            rospy.loginfo(message)
            return TriggerResponse(success=True, message=message)
        except Exception as e:
            error_msg = f"Failed to get current pose: {str(e)}"
            rospy.logerr(error_msg)
            return TriggerResponse(success=False, message=error_msg)
    
    def handle_command(self, msg):
        """处理简单命令 - 用于基础UI"""
        command = msg.data.strip().lower()
        
        if command.startswith("record:"):
            pos_name = command.split(":", 1)[1].strip()
            if pos_name:
                response = self.handle_record_position(SetStringRequest(data=pos_name))
                rospy.loginfo("Record response: %s", response.message)
            else:
                rospy.logwarn("Record command missing position name")
        
        elif command.startswith("move:"):
            pos_name = command.split(":", 1)[1].strip()
            if pos_name:
                response = self.handle_move_to_position(SetStringRequest(data=pos_name))
                rospy.loginfo("Move response: %s", response.message)
            else:
                rospy.logwarn("Move command missing position name")
        
        elif command == "list":
            response = self.handle_list_positions(TriggerRequest())
            rospy.loginfo("List response:\n%s", response.message)
        
        elif command.startswith("delete:"):
            pos_name = command.split(":", 1)[1].strip()
            if pos_name:
                response = self.handle_delete_position(SetStringRequest(data=pos_name))
                rospy.loginfo("Delete response: %s", response.message)
            else:
                rospy.logwarn("Delete command missing position name")
        
        elif command == "current":
            response = self.handle_get_current_pose(TriggerRequest())
            rospy.loginfo("Current pose:\n%s", response.message)
        
        else:
            rospy.logwarn("Unknown command: %s", command)
            rospy.loginfo("Available commands: record:<name>, move:<name>, list, delete:<name>, current")
    
    def run(self):
        """运行主循环"""
        rospy.loginfo("ArmTeachAndPlayback node running. Use services or publish to ~command topic.")
        rospy.spin()

if __name__ == "__main__":
    try:
        node = ArmTeachAndPlayback()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()