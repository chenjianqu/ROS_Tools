#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "def.h"

using namespace std;
namespace fs=std::filesystem;


struct SegImage{
    cv::Mat img0,img1;
    double img0_time,img1_time;
};

int main(int argc, char **argv)
{
    std::string file_name,save_dir;
    if(argc!=3){
        cerr<<"exit, usage:rosrun rosbag_test rosbag_test xxx.bag save_dir"<<endl;
        return 1;
    }
    else{
        file_name=argv[1],save_dir = argv[2];
    }

    fs::path left_save_path = fs::path(save_dir)/"cam0";
    fs::path right_save_path = fs::path(save_dir)/"cam1";

    if(!fs::is_directory(left_save_path) || !fs::is_directory(right_save_path)){
        cout<<left_save_path<<" or "<<right_save_path<<" is not a directory"<<endl;
        return -1;
    }

    ros::init(argc, argv, "rosbag_test");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    rosbag::Bag bag;
    bag.open(file_name, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        cerr<<"wrong,can not open "<<file_name<<endl;
        return 1;
    }

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back("/zed_cam/cam0");
    topics.emplace_back("/zed_cam/cam1");

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下

    int cnt=0;

    SegImage img;
    for(auto it=view.begin();it!=view.end();++it){
        auto topic=it->getTopic();
        sensor_msgs::Image::ConstPtr img_sensor=it->instantiate<sensor_msgs::Image>();
        double time=img_sensor->header.stamp.toSec();
        int seq_id = img_sensor->header.seq;
        cv_bridge::CvImageConstPtr ptr=cv_bridge::toCvShare(img_sensor,sensor_msgs::image_encodings::BGR8);

        string seq_id_str = PadNumber(seq_id,6);

        if(topic=="/zed_cam/cam0"){
            img.img0=ptr->image.clone();
            img.img0_time=time;
            cv::imwrite((left_save_path / (seq_id_str+".png")).string(),img.img0);
            cout<<(left_save_path / (seq_id_str+".png")).string()<<endl;
        }
        else if(topic=="/zed_cam/cam1"){
            img.img1=ptr->image.clone();
            img.img1_time=time;
            cv::imwrite((right_save_path / (seq_id_str+".png")).string(),img.img1);
            cout<<(right_save_path / (seq_id_str+".png")).string()<<endl;
        }
        cout<<time<<"  "<<topic<<endl;
    }
    bag.close();

    return 0;

}

