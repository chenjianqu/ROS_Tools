# rosbag_test

用于读取、解析、录制rosbag的相关工具。



### 读取VIODE数据集的bag文件，并保存图像
* 示例

```shell
rosrun rosbag_test get_image_from_bag /home/chen/Datasets/viode/3_high.bag /home/chen/tmp/
```





### 读取bag文件中的某个主题，并保存为图像

* 用法

```shell
rosrun rosbag_test write_image xxx.bag topic_name save_dir
```

* 示例

```shell
rosrun rosbag_test write_image \
/home/cjq/dataset/YT_gaojia/2022-10-23-13-00-04.bag \
/velodyne/image_mask \
/home/cjq/dataset/YT_gaojia/mask/
```



### 读取bag文件中的Mask和RGB图像，并保存到文件中

* 用法

```shell
rosrun rosbag_test write_rgb_mask xxx.bag rgb_topic mask_topic rgb_save_dir mask_save_dir
```

* 示例：

```shell
rosrun rosbag_test write_rgb_mask \
/home/cjq/dataset/YT_gaojia/2022-10-23-13-00-04.bag \
/velodyne/image \
/velodyne/image_mask \
/home/cjq/dataset/YT_gaojia/bag_rgb \
/home/cjq/dataset/YT_gaojia/bag_mask 
```





### 读取bag文件中的IMU消息，并保存到文件中

* 用法

```shell
rosrun rosbag_test get_imu_from_bag xxx.bag imu_topic save_path
```

* 示例：

```shell
bag_file=2022-10-17-15-41-09.bag
imu_topic=/met/imu
save_path=./imu.txt

rosrun rosbag_test get_imu_from_bag ${bag_file} ${imu_topic} ${save_path}
```





### 读取bag文件中的里程计消息，并保存到文件中

要求里程计消息的类型为`geometry_msgs/Vector3Stamped`

* 用法

```shell
rosrun rosbag_test get_odom_from_bag xxx.bag odom_topic save_path
```

* 示例：

```shell
bag_file=2022-10-17-15-41-09.bag
imu_topic=/met/wheel
save_path=./odom.txt

rosrun rosbag_test get_odom_from_bag ${bag_file} ${imu_topic} ${save_path}
```



### 读取bag文件中的图像时间戳，并保存到文件中

* 用法

```shell
rosrun rosbag_test get_image_time_stamp xxx.bag rgb_topic  save_path
```

* 示例：

```shell
bag_name=20230520_150407499_hq.bag
rosrun rosbag_test get_image_time_stamp ${bag_name} /camera/maxieye/610  ${bag_name}.txt

```











