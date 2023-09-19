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

#include <spdlog/fmt/fmt.h>


using namespace std;
namespace fs=std::filesystem;


int main(int argc, char **argv)
{
    if(argc!=4){
        cerr<<"usage:rosrun rosbag_test get_image_time_stamp xxx.bag rgb_topic time_stamp_save_path"<<endl;
        return 1;
    }

    std::string file_name,rgb_topic;
    file_name=argv[1], rgb_topic = argv[2] ;
    fs::path time_stamp_save_path(argv[3]);

    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    ros::init(argc, argv, "get_image_time_stamp"+string("_")+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    cout<<"Ready to read..."<<endl;

    rosbag::Bag bag;
    bag.open(file_name, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        cerr<<"wrong,can not open "<<file_name<<endl;
        return -1;
    }

    ofstream outfile(time_stamp_save_path,std::ios::out);

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back(rgb_topic);

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

            outfile<<fmt::format("{:.6f}",time)<<endl;
        }

        cout<<time<<"  "<<topic<<endl;
    }
    bag.close();

    outfile.close();

    return 0;
}

