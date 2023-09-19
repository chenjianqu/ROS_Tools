#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>

#include <opencv2/opencv.hpp>
#include "GeographicLib/LocalCartesian.hpp"
#include "utils/def.h"


using namespace std;
namespace fs=std::filesystem;


/**
 * 根据弢哥给的程序
 * @param yaw_angle
 * @param pitch_angle
 * @param roll_angle
 * @return
 */
Eigen::Quaterniond GnssYPR2Quatd(double yaw_angle,double pitch_angle,double roll_angle){
    Eigen::AngleAxisd pitch_(pitch_angle, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd roll_(roll_angle, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_(yaw_angle, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond Q = yaw_ * pitch_ * roll_;
    if (Q.w()<0)
        Q.coeffs() *= -1;
    Q.normalize();
    return Q;
}


int main(int argc, char **argv)
{
    if(argc!=4){
        cerr<<"usage:rosrun gps_convert write_gps_from_bag ${bag_path} ${gps_topic} ${save_path}"<<endl;
        return -1;
    }

    std::string file_name,gps_topic;
    file_name=argv[1], gps_topic = argv[2];
    fs::path save_path(argv[3]);
    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    ros::init(argc, argv, "write_gps_from_bag_"+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    cout<<"Ready to read..."<<endl;

    bool init;
    GeographicLib::LocalCartesian geoConverter;


    const string gnss_raw_path = save_path/"gnss_raw.txt";
    const string gnss_tum_path =save_path/"gnss_tum.txt";

    std::ofstream gnss_raw_file(gnss_raw_path, std::ios::out);
    std::ofstream gnss_tum_file(gnss_tum_path, std::ios::out);


    rosbag::Bag bag;
    bag.open(file_name, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        cerr<<"wrong,can not open "<<file_name<<endl;
        return -1;
    }

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back(gps_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下
    cout<<std::setiosflags(std::ios::fixed)<<std::setprecision(6);
    for(auto it=view.begin();it!=view.end();++it){
        if(!ros::ok()){
            break;
        }
        auto topic=it->getTopic();
        if(topic==gps_topic){
            sensor_msgs::NavSatFixConstPtr gps_msg_ptr = it->instantiate<sensor_msgs::NavSatFix>();


            string time_str = fmt::format("{:.6f}",gps_msg_ptr->header.stamp.toSec());
            ///保留10位小数
            gnss_raw_file<< std::setiosflags(std::ios::fixed) << std::setprecision(10)
                    <<time_str<<" "
                    <<gps_msg_ptr->latitude<<" "
                    <<gps_msg_ptr->longitude<<" "
                    <<gps_msg_ptr->altitude<<" "
                    <<(int)gps_msg_ptr->position_covariance_type<<" "
                    <<gps_msg_ptr->position_covariance[0]<<" "
                    <<gps_msg_ptr->position_covariance[1]<<" "
                    <<gps_msg_ptr->position_covariance[2]<<" "
                    <<gps_msg_ptr->position_covariance[3]<<" "
                    <<gps_msg_ptr->position_covariance[4]<<" "
                    <<gps_msg_ptr->position_covariance[5]<<" "
                    <<gps_msg_ptr->position_covariance[6]<<" "
                    <<gps_msg_ptr->position_covariance[7]<<" "
                    <<gps_msg_ptr->position_covariance[8]<<std::endl;



            //初始化
            if(!init){
                geoConverter.Reset(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude);
                init = true;
            }

            double point_xyz[3];
            geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude,
                                 point_xyz[0], point_xyz[1], point_xyz[2]);
            // geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, 0, xyz[0], xyz[1], xyz[2]);


            Quaterniond Q = GnssYPR2Quatd(-gps_msg_ptr->position_covariance[3] / 180. * M_PI,
                                          gps_msg_ptr->position_covariance[2] / 180. * M_PI,
                                          gps_msg_ptr->position_covariance[1] / 180. * M_PI );

            gnss_tum_file<<time_str<<" "<<point_xyz[0]<<" "<<point_xyz[1]<<" "<<point_xyz[2]<<" "
                    <<Q.x()<<" "<<Q.y()<<" "<<Q.z()<<" "<<Q.w()<<std::endl;

            std::cout <<time_str<<" lla:" << gps_msg_ptr->latitude << "," << gps_msg_ptr->longitude << ","
                      << gps_msg_ptr->altitude <<" | "<< point_xyz[0] << "," << point_xyz[1] << "," << point_xyz[2] << std::endl;
        }
    }

    bag.close();

    gnss_raw_file.close();
    gnss_tum_file.close();

    return 0;
}

