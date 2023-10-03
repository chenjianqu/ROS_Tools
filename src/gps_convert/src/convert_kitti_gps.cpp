#include <iostream>
#include <fstream>
#include <filesystem>
#include <regex>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <spdlog/spdlog.h>
#include <glog/logging.h>
#include <eigen3/Eigen/Eigen>

#include "GeographicLib/LocalCartesian.hpp"

namespace fs=std::filesystem;
using std::string;
using std::cout;
using std::endl;
using std::vector;


inline void split(const std::string& source, std::vector<std::string>& tokens, const std::string& delimiters = " ") {
    std::regex re(delimiters);
    std::copy(std::sregex_token_iterator(source.begin(), source.end(),re,-1),
              std::sregex_token_iterator(),
              std::back_inserter(tokens));
}


vector<double> StringLineToVector(const string& line){
    vector<string> tokens;
    if(line.find(',') != string::npos){
        split(line,tokens,",");
    }
    else{
        split(line,tokens," ");
    }

    vector<double> vec_data(tokens.size());
    for(int i=0;i<tokens.size();++i){
        try {
            vec_data[i] = std::stod(tokens[i]);
        }
        catch (std::exception &e){
            SPDLOG_ERROR("StringLineToVector() raise error,token:{},error:\n{}",tokens[i],e.what());
        }
    }
    return vec_data;
}


struct GNSS{
    double timestamp;
    double latitude;
    double longitude;
    double altitude;
    double roll;
    double pitch;
    double yaw;
};

constexpr double D2R = (M_PI / 180.0);
constexpr double R2D = (180.0 / M_PI);


bool init;
GeographicLib::LocalCartesian geoConverter;

std::ofstream out_file;

void gpsCallback(const GNSS& gnss)
{
    //初始化
    if(!init){
        geoConverter.Reset(gnss.latitude, gnss.longitude, gnss.altitude);
        init = true;
    }

    double point_xyz[3];
    geoConverter.Forward(gnss.latitude, gnss.longitude,gnss.altitude,
                         point_xyz[0], point_xyz[1], point_xyz[2]);
    // geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, 0, xyz[0], xyz[1], xyz[2]);

    //std::cout << "lla:" << gps_msg_ptr->latitude << "," << gps_msg_ptr->longitude << ","
    //<< gps_msg_ptr->altitude <<" | "<< point_xyz[0] << "," << point_xyz[1] << "," << point_xyz[2] << std::endl;


    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(gnss.roll,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(gnss.pitch,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(gnss.yaw,Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;


    std::stringstream ss;
    ss << std::setiosflags(std::ios::fixed) << std::setprecision(10) << gnss.timestamp;

    out_file<<ss.str()<<" "<<point_xyz[0]<<" "<<point_xyz[1]<<" "<<point_xyz[2]<<" "
            <<quaternion.x()<<" "<<quaternion.y()<<" "<<quaternion.z()<<" "<<quaternion.w()<<std::endl;
}






int main(int argc,char **argv)
{
    if(argc!=3){
        std::cerr<<"Usage:rosrun gps_convert convert_kitti_gps ${read_path} ${save_path}"<<std::endl;
        return -1;
    }

    std::string read_path(argv[1]);
    fs::path save_path(argv[2]);

    if(!fs::exists(save_path.parent_path())){
        std::cerr<<"save_path.parent_path() is not exist"<<std::endl;
        return -1;
    }

    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    init = false;
    ros::init(argc,argv,"convert_kitti_gps_"+time_str);
    ros::NodeHandle n;

    out_file.open(save_path, std::ios::out);

    vector<fs::path> txt_path;
    for (const auto& entry : fs::directory_iterator(read_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            txt_path.push_back(entry.path());
        }
    }
    std::sort(txt_path.begin(),txt_path.end());

    for (const auto& path : txt_path) {
        cout<<"read:"<<path<<endl;
        std::ifstream infile(path);
        string line;
        while (getline(infile,line)){
            vector<double> token = StringLineToVector(line);
            CHECK(token.size()==30);
            GNSS gnss;
            gnss.timestamp = std::stod(path.filename().stem());
            gnss.latitude = token[0];
            gnss.longitude = token[1];
            gnss.altitude = token[2];
            gnss.roll = token[3];
            gnss.pitch = token[4];
            gnss.yaw = token[5];

            gpsCallback(gnss);
        }
        infile.close();
    }

    out_file.close();

    return 0;
}

