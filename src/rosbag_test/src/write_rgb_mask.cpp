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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


using namespace std;
namespace fs=std::filesystem;


int main(int argc, char **argv)
{
    if(argc!=6){
        cerr<<"usage:rosrun rosbag_test write_rgb_mask xxx.bag rgb_topic mask_topic rgb_save_dir mask_save_dir"<<endl;
        return 1;
    }

    std::string file_name,rgb_topic,mask_topic;
    file_name=argv[1], rgb_topic = argv[2] , mask_topic = argv[3];
    fs::path rgb_save_dir(argv[4]);
    fs::path mask_save_dir(argv[5]);

    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    ros::init(argc, argv, "write_rgb_mask_"+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    cout<<"Ready to read..."<<endl;

    rosbag::Bag bag;
    bag.open(file_name, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        cerr<<"wrong,can not open "<<file_name<<endl;
        return -1;
    }

    if(!fs::exists(rgb_save_dir)){
        fs::create_directory(rgb_save_dir);
    }
    if(!fs::exists(mask_save_dir)){
        fs::create_directory(mask_save_dir);
    }

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back(rgb_topic);
    topics.emplace_back(mask_topic);

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

        if(topic==rgb_topic){
            cv_bridge::CvImageConstPtr ptr=cv_bridge::toCvShare(img_sensor,sensor_msgs::image_encodings::BGR8);
            cv::Mat img=ptr->image.clone();
            cv::imwrite((rgb_save_dir/name).string(),img);
        }
        else if(topic==mask_topic){
            cv_bridge::CvImageConstPtr ptr=cv_bridge::toCvShare(img_sensor,sensor_msgs::image_encodings::MONO8);
            cv::Mat img=ptr->image.clone();
            name += ".png";//COLMAP要求在rgb图像名字的基础上增加".png"
            cv::imwrite((mask_save_dir/name).string(),img);
        }
        cout<<time<<"  "<<topic<<endl;
    }
    bag.close();

    return 0;
}

