#include <iostream>
#include <memory>


#include <ros/ros.h>

#include <std_msgs/Time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>


using namespace std;

std::shared_ptr<ros::Publisher> pub;





void PointCloudCallBack(const sensor_msgs::PointCloud2 &msg){
    pub->publish(msg);
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");//防止中文乱码
    ros::init(argc, argv, "relay_pointcloud_node");//初始化节点
    ros::start();//启动节点
    ros::NodeHandle nh;

    ros::Subscriber sub=nh.subscribe("/pcl_out",3,PointCloudCallBack);
    pub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>("cloud",5)));
    ros::Publisher obj_pub=nh.advertise<visualization_msgs::Marker>("obj_topic",10);


    ROS_INFO_STREAM("节点初始化完成");

    bool isPoseUpdate=false;
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        visualization_msgs::Marker marker;
        obj_pub.publish(marker);
        ros::spinOnce();
        loop_rate.sleep();
    }


    ros::shutdown();
    ROS_INFO_STREAM("关闭节点");
    return 0;
}




