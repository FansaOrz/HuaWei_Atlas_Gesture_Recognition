#!/bin/sh

# 开启rslidar以及rviz
gnome-terminal -x bash -c "cd ~/aloam; source devel/setup.bash; roslaunch aloam_velodyne display.launch;"

# 开启保存地图终端
gnome-terminal -x bash -c "bash ./save_map.sh"

# 开启终端进行ssh控制Atlas
expect ./mapping_connection.sh
