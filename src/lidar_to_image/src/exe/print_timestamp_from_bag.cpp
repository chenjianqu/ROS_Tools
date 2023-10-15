//
// Created by cjq on 23-9-20.
//
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <filesystem>

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <gflags/gflags.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include "pose_type.h"

using namespace std;
namespace fs=std::filesystem;


DEFINE_string(bag_path,"","bag_path");
DEFINE_string(lidar_topic,"/lidar/hs/pandar64/topmiddle","lidar_topic");
DEFINE_string(image_topic,"/cam/met/611/front","image_topic");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    if(FLAGS_bag_path.empty()){
        SPDLOG_ERROR("-bag_path is missing");
        return -1;
    }
    if(!fs::exists(FLAGS_bag_path)){
        SPDLOG_ERROR("{} not exists!",FLAGS_bag_path);
        return -1;
    }

    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
    ros::init(argc, argv, "print_timestamp_from_bag_"+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    cout<<"Ready to read..."<<endl;
    rosbag::Bag bag;
    bag.open(FLAGS_bag_path, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        SPDLOG_ERROR("Open {} failed!",FLAGS_bag_path);
        return -1;
    }

    vector<string> timestamps;

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back(FLAGS_lidar_topic);
    topics.emplace_back(FLAGS_image_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下

    cout<<"running"<<endl;

    for(auto it=view.begin();it!=view.end();++it){
        if(!ros::ok()){
            break;
        }
        auto topic=it->getTopic();
        if(topic == FLAGS_lidar_topic){
            sensor_msgs::PointCloud2ConstPtr msg = it->instantiate<sensor_msgs::PointCloud2>();
            timestamps.emplace_back(fmt::format("{:.6f} lidar",msg->header.stamp.toSec()));
        }
        else if(topic == FLAGS_image_topic){
            sensor_msgs::ImageConstPtr msg = it->instantiate<sensor_msgs::Image>();
            timestamps.emplace_back(fmt::format("{:.6f} image",msg->header.stamp.toSec()));
        }
    }
    bag.close();

    std::sort(timestamps.begin(),timestamps.end());

    cout<<"writing"<<endl;

    std::ofstream fout("./timestamps.txt");
    for(auto& timestamp:timestamps){
        fout<<timestamp<<endl;
    }
    fout.close();

    cout<<"finished"<<endl;

    return 0;
}
