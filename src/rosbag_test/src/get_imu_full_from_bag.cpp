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
#include <sensor_msgs/Imu.h>

#include <spdlog/fmt/fmt.h>


using namespace std;
namespace fs=std::filesystem;


int main(int argc, char **argv)
{
    if(argc!=4){
        cerr<<"usage:rosrun rosbag_test get_imu_full_from_bag xxx.bag imu_topic save_path"<<endl;
        return 1;
    }

    std::string bag_file_name,imu_topic;
    bag_file_name=argv[1], imu_topic = argv[2];
    fs::path save_path(argv[3]);

    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    ros::init(argc, argv, "get_imu_full_from_bag_"+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    cout<<"Ready to read..."<<endl;

    rosbag::Bag bag;
    bag.open(bag_file_name, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        cerr<<"wrong,can not open "<<bag_file_name<<endl;
        return -1;
    }

    ofstream outfile(save_path.string(),std::ios::out);

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back(imu_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下
    cout<<std::setiosflags(std::ios::fixed)<<std::setprecision(6);
    for(auto it=view.begin();it!=view.end();++it){
        if(!ros::ok()){
            break;
        }
        auto topic=it->getTopic();
        if(topic == imu_topic){
            sensor_msgs::Imu::ConstPtr imu_msg = it->instantiate<sensor_msgs::Imu>();
            outfile<<fmt::format("{:.9f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}",
                                 imu_msg->header.stamp.toSec(),
                                 imu_msg->angular_velocity.x,
                                 imu_msg->angular_velocity.y,
                                 imu_msg->angular_velocity.z,
                                 imu_msg->linear_acceleration.x,
                                 imu_msg->linear_acceleration.y,
                                 imu_msg->linear_acceleration.z
                                 )<<std::endl;

            cout<<imu_msg->header.stamp.toSec()<<endl;
        }
    }
    outfile.close();
    bag.close();

    return 0;
}

