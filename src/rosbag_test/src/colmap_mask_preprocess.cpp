//
// Created by cjq on 23-7-26.
//
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <filesystem>
#include <cstdio>
#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


using namespace std;
namespace fs=std::filesystem;

vector<int> class_ids(256,1);

cv::Mat manual_mask;

cv::Mat PreprocessMask(const cv::Mat &input_mask){
    int rows = input_mask.rows;
    int cols = input_mask.cols;

    CHECK_EQ(rows,manual_mask.rows);
    CHECK_EQ(cols,manual_mask.cols);

    static cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));


    //创建 mask of car and sky
    cv::Mat save_mask = cv::Mat(input_mask.size(),input_mask.type(),
                                cv::Scalar(255));

    ///设置要mask掉的类别，车辆
    class_ids[1]=0;

    for(int i=0;i<rows;++i){
        for(int j=0;j<cols;++j){
            uchar pixel = input_mask.at<uchar>(i,j);
            if(class_ids[pixel] == 0){
                save_mask.at<uchar>(i,j) = 0;
            }
        }
    }

    ///进行erode
    cv::erode(save_mask, save_mask, element);

    ///设置要mask掉的类别，sky
    class_ids[3]=0;
    for(int i=0;i<rows;++i){
        for(int j=0;j<cols;++j){
            uchar pixel = input_mask.at<uchar>(i,j);
            if(class_ids[pixel] == 0 || manual_mask.at<uchar>(i,j)==0){
                save_mask.at<uchar>(i,j) = 0;
            }
        }
    }

    return save_mask;
}



DEFINE_string(bag_path, "", "bag_path");
DEFINE_string(rgb_topic_name, "", "rgb_topic_name");
DEFINE_string(mask_topic_name, "", "mask_topic_name");
DEFINE_string(rgb_save_dir, "", "rgb_save_dir");
DEFINE_string(mask_save_dir, "", "mask_save_dir");
DEFINE_string(manual_mask_path, "", "manual_mask_path");


int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_bag_path.empty()) <<"-bag_path is missing!";
    CHECK(!FLAGS_rgb_topic_name.empty()) <<"-rgb_topic_name is missing!";
    CHECK(!FLAGS_mask_topic_name.empty()) <<"-mask_topic_name is missing!";
    CHECK(!FLAGS_rgb_save_dir.empty()) <<"-rgb_save_dir is missing!";
    CHECK(!FLAGS_mask_save_dir.empty()) <<"-mask_save_dir is missing!";
    CHECK(!FLAGS_manual_mask_path.empty()) <<"-manual_mask_path is missing!";

    fs::path rgb_save_dir(FLAGS_rgb_save_dir);
    fs::path mask_save_dir(FLAGS_mask_save_dir);

    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    ros::init(argc, argv, "colmap_mask_preprocess_"+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(!fs::exists(FLAGS_manual_mask_path)){
        cerr<<FLAGS_manual_mask_path<<" not exists!"<<endl;
        return -1;
    }

    manual_mask = cv::imread(FLAGS_manual_mask_path,0);

    if(!fs::exists(FLAGS_bag_path)){
        cerr<<FLAGS_bag_path<<" not exists!"<<endl;
        return -1;
    }

    cout<<"Ready to read..."<<endl;

    rosbag::Bag bag;
    bag.open(FLAGS_bag_path, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        cerr<<"wrong,can not open "<<FLAGS_bag_path<<endl;
        return -1;
    }

    if(!fs::exists(rgb_save_dir)){
        fs::create_directory(rgb_save_dir);
    }
    if(!fs::exists(mask_save_dir)){
        fs::create_directory(mask_save_dir);
    }

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back(FLAGS_rgb_topic_name);
    topics.emplace_back(FLAGS_mask_topic_name);

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下
    cout<<std::setiosflags(std::ios::fixed)<<std::setprecision(6);
    for(auto it=view.begin();it!=view.end();++it){
        if(!ros::ok()){
            break;
        }
        auto topic=it->getTopic();
        sensor_msgs::Image::ConstPtr img_sensor=it->instantiate<sensor_msgs::Image>();
        double time=img_sensor->header.stamp.toSec();

        string name = to_string(time)+".png";

        if(topic==FLAGS_rgb_topic_name){
            cv_bridge::CvImageConstPtr ptr=cv_bridge::toCvShare(img_sensor,sensor_msgs::image_encodings::BGR8);
            cv::Mat img=ptr->image.clone();
            cv::imwrite((rgb_save_dir/name).string(),img);
        }
        else if(topic==FLAGS_mask_topic_name){
            cv_bridge::CvImageConstPtr ptr=cv_bridge::toCvShare(img_sensor,sensor_msgs::image_encodings::MONO8);
            cv::Mat img=ptr->image.clone();

            img = PreprocessMask(img);

            name += ".png";//COLMAP要求在rgb图像名字的基础上增加".png"
            cv::imwrite((mask_save_dir/name).string(),img);
        }
        cout<<time<<"  "<<topic<<endl;
    }
    bag.close();

    return 0;
}

