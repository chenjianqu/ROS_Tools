# gps_convert


## gps_pose

将gps原始的经纬度数据转换为ENU坐标系,并保存到文件。

* 用法：

```
rosrun gps_convert gps_pose ${gps_topic} ${save_path}
```

并播放bag

* 示例：

```shell
gps_topic=/met/gps/lla
save_path=./gnss.txt

rosrun gps_convert gps_pose ${gps_topic} ${save_path}
```





### gps_record

订阅GPS话题，并将结果直接保存到文件中。

* 用法：

```shell
rosrun gps_convert gps_record ${gps_topic} ${save_path}
```

并播放bag

* 示例：

```shell
gps_topic=/met/gps/lla
save_path=./gnss_raw.txt

rosrun gps_convert gps_record ${gps_topic} ${save_path}
```





### write_gps_from_bag

直接读取.bag文件，解析其中的GPS消息，并保存到txt文件中，保存格式有原始的LLA格式和TUM格式。

* 用法

```shell
rosrun gps_convert write_gps_from_bag xxx.bag gps_topic
```



* 示例

```shell
bag_file=2022-10-17-12-09-28.bag
gps_topic=/met/gps/lla
save_path=.

rosrun gps_convert write_gps_from_bag ${bag_file} ${gps_topic} ${save_path}
```





