# ROS_Tools
这个仓库存放着用于测试的ROS代码。  





## rosbag_test

### cvt_met_to_bag

```shell
catkin_make
```



```shell
sourced

rosbag_path=/mnt/logs/07_MTD/SL_003/group2/routine13/1229-1/MET-1/output/rtkimu_hc_220pro/
output_path=/media/cjq/新加卷/datasets/MemoryDriving/SL-003/routine13/1229-1_1/rtk

mkdir -p ${output_path}
rosrun rosbag_test cvt_met_to_bag ${rosbag_path} ${output_path}

```

