#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <spdlog/fmt/fmt.h>


using namespace std;
namespace fs=std::filesystem;


int main(int argc, char **argv)
{
    if(argc!=4){
        cerr<<"usage:rosrun rosbag_test get_odom_from_bag xxx.bag odom_topic save_path"<<endl;
        return 1;
    }

    std::string bag_file_name = argv[1];
    std::string odom_topic = argv[2];
    fs::path save_path(argv[3]);

    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    ros::init(argc, argv, "get_odom_from_bag_"+time_str);
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
    topics.emplace_back(odom_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下
    cout<<std::setiosflags(std::ios::fixed)<<std::setprecision(6);
    for(auto it=view.begin();it!=view.end();++it){
        if(!ros::ok()){
            break;
        }
        auto topic=it->getTopic();
        if(topic == odom_topic){
            geometry_msgs::Vector3Stamped::ConstPtr odom_msg = it->instantiate<geometry_msgs::Vector3Stamped>();
            outfile<<fmt::format("{:.9f} {:.6f} {:.6f} {:.6f}",
                                 odom_msg->header.stamp.toSec(),
                                 odom_msg->vector.x,
                                 odom_msg->vector.y,
                                 odom_msg->vector.z
                                 )<<std::endl;

            cout<<odom_msg->header.stamp.toSec()<<endl;
        }
    }
    outfile.close();
    bag.close();

    return 0;
}

