#!/bin/bash

# 加载 ROS 工作空间环境
#source /home/jc/a1_ws/devel/setup.bash

# 启动 roscore
gnome-terminal --title="roscore" -- bash -c "roscore; exec bash"

# 等待 2 秒，确保 roscore 起来
sleep 2

# 启动各个功能节点
rosrun iimt_moveit_dual robot_control.py &
rosrun iimt_moveit_dual full_body_joint_publisher.py &

# 启动主 launch 文件
gnome-terminal --title="Control Demo" -- bash -c "
  export ROSCONSOLE_CONFIG_FILE=/home/jc/a1_ws/src/iimt_moveit_dual/scripts/rosconsole.config &&
  roslaunch iimt_moveit_dual demo.launch fake_execution:=false;
  exec bash
"
sleep 35

#fk的启动
gnome-terminal --title="listen action fk" -- bash -c "
  export ROSCONSOLE_CONFIG_FILE=/home/jc/a1_ws/src/iimt_moveit_dual/scripts/rosconsole.config &&
  rosrun iimt_moveit_dual listen_action_fk.py;
  exec bash
"