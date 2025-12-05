#!/bin/bash

# 启动左臂
gnome-terminal --title="ARM_L" -- bash -c "
sudo '$HOME/Documents/arm_l/build/user/MIT_Controller/arm_l' 3 r;
echo 'ARM_L exited.';
exec bash
" &

# 启动右臂
gnome-terminal --title="ARM_R" -- bash -c "
sudo '$HOME/Documents/arm_r/build/user/MIT_Controller/arm_r' 3 r;
echo 'ARM_R exited.';
exec bash
" &

echo "Waiting 10 seconds for drivers to initialize..."
sleep 10