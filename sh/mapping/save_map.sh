#!/bin/bash



#输入save时才保存地图
echo "INPUT: save (when you want to save the grid map and the point cloud)"
read COMMAND
case "$COMMAND" in
    "save")
        echo "YYYYYY"
        cd 
        source /home/robot/Desktop/cpu_mapping_rs/devel/setup.bash
        rosrun map_server map_saver
        rosbag record -O out_pcd /globalmap/map_full
        rosrun pcl_ros bag_to_pcd out_pcd.bag /globalmap/map_full .
        rm ./out_pcd.bag
        mv 0.000000000.pcd map.pcd
        ;;
    *)
        echo "wrong"
        ;;
esac
