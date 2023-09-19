//
// Created by chen on 2022/1/4.
//

#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

struct SegImage{
    cv::Mat img0,img0_seg,img1,img1_seg;
    double img0_time,img0_seg_time,img1_time,img1_seg_time;
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
    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
    ros::init(argc, argv, "get_image_from_bag_"+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    rosbag::Bag bag;
    bag.open(file_name, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        cerr<<"wrong,can not open "<<file_name<<endl;
        return 1;
    }

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back("/cam0/image_raw");
    topics.emplace_back("/cam0/segmentation");
    topics.emplace_back("/cam1/image_raw");
    topics.emplace_back("/cam1/segmentation");

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下

    int cnt=0;

    SegImage img;
    for(auto it=view.begin();it!=view.end();++it){
        auto topic=it->getTopic();
        sensor_msgs::Image::ConstPtr img_sensor=it->instantiate<sensor_msgs::Image>();
        double time=img_sensor->header.stamp.toSec();
        cv_bridge::CvImageConstPtr ptr=cv_bridge::toCvShare(img_sensor,sensor_msgs::image_encodings::BGR8);

        if(topic=="/cam0/image_raw"){
            img.img0=ptr->image.clone();
            img.img0_time=time;
            cv::imwrite(save_dir+"/"+to_string(time)+"_cam0.png",img.img0);
        }
        else if(topic=="/cam1/image_raw"){
            img.img1=ptr->image.clone();
            img.img1_time=time;
            cv::imwrite(save_dir+"/"+to_string(time)+"_cam1.png",img.img1);
        }
        else if(topic=="/cam0/segmentation"){
            img.img0_seg=ptr->image.clone();
            img.img0_seg_time=time;
            cv::imwrite(save_dir+"/"+to_string(time)+"_seg0.png",img.img0_seg);
        }
        else if(topic=="/cam1/segmentation"){
            img.img1_seg=ptr->image.clone();
            img.img1_seg_time=time;
            cv::imwrite(save_dir+"/"+to_string(time)+"_seg1.png",img.img1_seg);
        }
        cout<<time<<"  "<<topic<<endl;
    }
    bag.close();

    return 0;

}

