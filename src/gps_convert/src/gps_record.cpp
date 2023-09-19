#include <iostream>
#include <fstream>
#include <filesystem>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "utils/def.h"


namespace fs=std::filesystem;

bool init;

std::ofstream out_file;

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
{
    string time_str = fmt::format("{:.6f}",gps_msg_ptr->header.stamp.toSec());

    ///保留10位小数
    out_file<< std::setiosflags(std::ios::fixed) << std::setprecision(10)
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

    std::cout <<time_str<< " " << gps_msg_ptr->latitude << "," << gps_msg_ptr->longitude << ","
    << gps_msg_ptr->altitude <<std::endl;
}


int main(int argc,char **argv)
{
    if(argc!=3){
        std::cerr<<"Usage:rosrun gps_convert gps_record ${gps_topic} ${save_path}"<<std::endl;
        return -1;
    }

    std::string gps_topic(argv[1]);
    fs::path save_path(argv[2]);

    if(!fs::exists(save_path.parent_path())){
        std::cerr<<"save_path.parent_path() is not exist"<<std::endl;
        return -1;
    }
    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
    init = false;
    ros::init(argc,argv,"gps_record_subscriber_"+time_str);
    ros::NodeHandle n;

    out_file.open(save_path, std::ios::out);

    cout<<"wait gps msg..."<<endl;

    ros::Subscriber pose_sub=n.subscribe(gps_topic,10,gpsCallback);

    ros::spin();

    out_file.close();

    return 0;
}

