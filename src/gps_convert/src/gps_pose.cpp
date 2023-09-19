#include <iostream>
#include <fstream>
#include <filesystem>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "GeographicLib/LocalCartesian.hpp"

namespace fs=std::filesystem;

bool init;
GeographicLib::LocalCartesian geoConverter;

std::ofstream out_file;

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
{
    //初始化
    if(!init){
        geoConverter.Reset(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude);
        init = true;
    }

    double point_xyz[3];
    geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude,
                         point_xyz[0], point_xyz[1], point_xyz[2]);
    // geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, 0, xyz[0], xyz[1], xyz[2]);

    std::cout << "lla:" << gps_msg_ptr->latitude << "," << gps_msg_ptr->longitude << ","
    << gps_msg_ptr->altitude <<" | "<< point_xyz[0] << "," << point_xyz[1] << "," << point_xyz[2] << std::endl;

    std::stringstream ss;
    ss << std::setiosflags(std::ios::fixed) << std::setprecision(6) << gps_msg_ptr->header.stamp.toSec();

    out_file<<ss.str()<<" "<<point_xyz[0]<<" "<<point_xyz[1]<<" "<<point_xyz[2]<<" "
            <<0<<" "<<0<<" "<<0<<" "<<1<<std::endl;
}






int main(int argc,char **argv)
{
    if(argc!=3){
        std::cerr<<"Usage:rosrun gps_convert gps_pose ${gps_topic} ${save_path}"<<std::endl;
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
    ros::init(argc,argv,"gps_subscriber_"+time_str);
    ros::NodeHandle n;

    out_file.open(save_path, std::ios::out);

    ros::Subscriber pose_sub=n.subscribe(gps_topic,10,gpsCallback);

    ros::spin();

    out_file.close();

    return 0;
}

