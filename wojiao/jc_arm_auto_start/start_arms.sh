#!/bin/bash

# 日志目录
LOG_DIR="/home/jc/asser/arms_logs"
mkdir -p "$LOG_DIR"

# 启动左臂（后台运行，输出到日志）
"/home/jc/Documents/arm_l/build/user/MIT_Controller/arm_l" 3 r >> "$LOG_DIR/arm_l.log" 2>&1 &

# 启动右臂（后台运行，输出到日志）
"/home/jc/Documents/arm_r/build/user/MIT_Controller/arm_r" 3 r >> "$LOG_DIR/arm_r.log" 2>&1 &

# 可选：等待一下确保启动
sleep 5

echo "$(date): Arms started." >> "$LOG_DIR/startup.log"
