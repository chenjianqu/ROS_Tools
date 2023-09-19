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

using namespace std;


int main(int argc, char **argv)
{
    std::string file_name,save_dir,topic_name;
    if(argc!=4){
        cerr<<"usage:rosrun rosbag_test write_image xxx.bag topic_name save_dir"<<endl;
        return 1;
    }
    else{
        file_name=argv[1], topic_name = argv[2] , save_dir = argv[3];
    }
    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    ros::init(argc, argv, "write_image_"+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(!std::filesystem::exists(save_dir)){
        std::filesystem::create_directory(save_dir);
    }

    rosbag::Bag bag;
    bag.open(file_name, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        cerr<<"wrong,can not open "<<file_name<<endl;
        return 1;
    }

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back(topic_name);

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下
    cout<<std::setiosflags(std::ios::fixed)<<std::setprecision(6);
    for(auto it=view.begin();ros::ok() && it!=view.end();++it){
        auto topic=it->getTopic();
        sensor_msgs::Image::ConstPtr img_sensor=it->instantiate<sensor_msgs::Image>();
        double time=img_sensor->header.stamp.toSec();

        cv_bridge::CvImageConstPtr ptr=cv_bridge::toCvShare(img_sensor,img_sensor->encoding);

        if(topic==topic_name){
            cv::Mat img=ptr->image.clone();
            cv::imwrite(save_dir+"/"+to_string(time)+".png",img);
        }
        cout<<time<<"  "<<topic<<endl;
    }
    bag.close();

    return 0;
}

