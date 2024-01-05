/*
 * @Author: xwding 995481933@qq.com
 * @Date: 2023-02-24 10:37:36
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-01-03 13:24:03
 * @FilePath: /catkin_ws_LIO/src/rosbagtest/src/rosbagread.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 *
 */

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "HC220CAN.h"
#include "rtklib.h"

//#include"abac.h"
// #include"rtklib.h"
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/search/impl/search.hpp>
// #include <pcl/range_image/range_image.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
// #include <pcl/registration/icp.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl_conversions/pcl_conversions.h>

//#include "utility.h"
#include <utility>
//#include "rtklib.h"

// static constexpr auto TOPIC_WHEEL = "/met/wheel";
// static constexpr auto TOPIC_POINTSCLOUD = "/points_raw";
// static constexpr auto TOPIC_IMU = "/imu_raw";
// static constexpr auto TOPIC_ODO = "/imu_raw";
// static constexpr auto TOPIC_HuaceIMU = "/met/imu";
// static constexpr auto TOPIC_HuaceGNSS = "/met/gps/lla";
// static constexpr auto TOPIC_HuaceGNSSVEL = "/met/gps/velocity";
// static constexpr auto TOPIC_WHEEL = "/wheel/sick/right_back/speed_float32";
//  static constexpr auto TOPIC_HuaceIMU = "/met/imu_hc220pro";
//  static constexpr auto TOPIC_HuaceGNSS = "/met/gps/lla_hc220pro";
//   static constexpr auto TOPIC_HuaceGNSSVEL = "/met/gps/velocity_hc220pro";
//  static constexpr auto TOPIC_WHEEL = "/met/wheel";
// // static constexpr auto TOPIC_WHEEL = "/wheel/sick/dfs60/right";
// static constexpr auto TOPIC_CAMERA = "/camera/maxieye/610";


//  static constexpr auto TOPIC_HuaceIMU = "/imu0";
//  static constexpr auto TOPIC_HuaceGNSS = "/gnss0";
// static constexpr auto TOPIC_HuaceGNSSVEL = "/met/gps/velocity_hc220pro";
// static constexpr auto TOPIC_CAMERA = "/cam0";



//  static constexpr auto TOPIC_HuaceIMU = "/imu0";
//  static constexpr auto TOPIC_HuaceGNSS = "/gnss0";
// static constexpr auto TOPIC_HuaceGNSSVEL = "/met/gps/velocity_hc220pro";
// static constexpr auto TOPIC_CAMERA = "/cam0";

  static constexpr auto TOPIC_HuaceIMU = "/rtkimu/hc/220pro/imu";
  static constexpr auto TOPIC_HuaceGNSS = "/rtkimu/hc/220pro/nav";
  static constexpr auto TOPIC_HuaceGNSSVEL = "/rtkimu/hc/220pro/velocity";
// //static constexpr auto TOPIC_WHEEL = "/met/wheel";
// static constexpr auto TOPIC_WHEEL = "/wheel/sick/dfs60/right/art";
static constexpr auto TOPIC_WHEEL = "/veh/vehicleinfo/pulse/right/rr";
static constexpr auto TOPIC_CAMERA = "/cam0";
static constexpr auto TOPIC_HC220PRO_381 = "/rtkimu/hc/220pro/0x381";
static constexpr auto TOPIC_HC220PRO_382 = "/rtkimu/hc/220pro/0x382";
static constexpr auto TOPIC_HC220PRO_383 = "/rtkimu/hc/220pro/0x383";
static constexpr auto TOPIC_HC220PRO_385 = "/rtkimu/hc/220pro/0x385";
static constexpr auto TOPIC_HC220PRO_386 = "/rtkimu/hc/220pro/0x387";
static constexpr auto TOPIC_HC220PRO_388 = "/rtkimu/hc/220pro/0x388";
static constexpr auto TOPIC_HC220PRO_389 = "/rtkimu/hc/220pro/0x389";
static constexpr auto TOPIC_HC220PRO_38d = "/rtkimu/hc/220pro/0x38d";

//  static constexpr auto TOPIC_HuaceIMU = "/met/imu";
//  static constexpr auto TOPIC_HuaceGNSS = "/met/gps/lla";
//   static constexpr auto TOPIC_HuaceGNSSVEL = "/met/gps/velocity";
// //static constexpr auto TOPIC_WHEEL = "/met/wheel";
// static constexpr auto TOPIC_WHEEL = "/wheel/sick/dfs60/right";
// // static constexpr auto TOPIC_CAMERA = "/cam0";

#include <stdio.h>
#include <string.h>
#include <dirent.h>

 struct VelodynePointXYZIRT
 {
     PCL_ADD_POINT4D
     PCL_ADD_INTENSITY;
     uint16_t ring;
     float time;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 } EIGEN_ALIGN16;
 POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))
 using PointXYZIRT = VelodynePointXYZIRT;

using namespace std;

void getFiles(std::string path, std::vector<std::string> &files)
{
    DIR *dir;
    struct dirent *ptr;
    if ((dir = opendir(path.c_str())) == NULL)
    {
        perror("Open dir error...");
        return;
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) /// current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8) /// file
        {
            std::string strFile;
            strFile = path;
            // strFile += "/";
            strFile += ptr->d_name;
            files.push_back(strFile);
        }
        else
        {
            continue;
        }
    }
    closedir(dir);
}
void GetFileName(char *path, char *filename)
{
    char *ptr = NULL;
    ptr = strrchr(path, '/');
    if (!ptr)
        return;
    memcpy(filename, ptr + 1, strlen(ptr + 1));
}

bool cmp(const pair<string, int> &x, const pair<string, int> &y)
{
    return x.second > y.second;
}

void sortMapByValue(map<string, int> &tMap, vector<pair<string, int>> &tVector)
{
    for (map<string, int>::iterator curr = tMap.begin(); curr != tMap.end(); curr++)
        tVector.push_back(make_pair(curr->first, curr->second));

    sort(tVector.begin(), tVector.end(), cmp);
}

int saveGNSSFILE(FILE *pf, sensor_msgs::NavSatFix::Ptr plla,
                 geometry_msgs::Vector3Stamped::Ptr pvel)
{
    static sensor_msgs::NavSatFix::ConstPtr llaLast = nullptr;
    static geometry_msgs::Vector3Stamped::ConstPtr velLast = nullptr;
    if (llaLast == nullptr)
    {
    }
    int flag = 0;
    if (plla && pvel)
    {
        flag = 1;
    }
    if (flag)
    {
        static double lastgpssecond = 0;
        double second = plla->header.stamp.toSec();
        if (second > lastgpssecond)
        {
            fprintf(pf, "%10.4f %13.9f %13.9f %13.4f %13.4f  %13.4f  %13.4f  %13.4f  %13.4f  %13.4f %d\n",
                    second, plla->latitude, plla->longitude, plla->altitude,
                    pvel->vector.x, pvel->vector.y, pvel->vector.z,
                    plla->position_covariance[1], plla->position_covariance[2], plla->position_covariance[3],
                    plla->position_covariance_type);
            fflush(pf);

            plla = NULL;
            pvel = NULL;
            lastgpssecond = second;
        }
    }

    // if (plla)
    // {
    //     static double lastgpssecond = 0;
    //     double second = plla->header.stamp.toSec();
    //     if (second > lastgpssecond)
    //     {
    //         fprintf(pf, "%10.4f %13.9f %13.9f %13.4f 0.0  0.0  0.0  %13.4f  %13.4f  %13.4f 1\n",
    //                 second, plla->latitude, plla->longitude, plla->altitude,
                    
    //                 plla->position_covariance[0], plla->position_covariance[4], plla->position_covariance[8]);
    //         fflush(pf);

    //         plla = NULL;
    //         pvel = NULL;
    //         lastgpssecond = second;
    //     }
    // }
    return flag;
}


int save220msg381_382(FILE *pf, std::shared_ptr< Msg381> p381,
                  std::shared_ptr<Msg382> p382)
{
    int flag = 0;
    if (p381 && p382)
    {
        flag = 1;
    }
    if (flag)
    {
        static double lastgpssecond = 0;
        
        double second = p381->MON_GPS_second;
        if (second > lastgpssecond && fabs(p381->MON_GPS_second-p382->MON_GPS_second)<1e-1)
        {
            fprintf(pf, "%10.4f, %10.7f, %10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f \n",
                    second, p381->MON_install_imu2car_pitch,p381->MON_install_imu2car_roll, p381->MON_install_imu2car_yaw,
                    p382->MON_gyo_bias_x,p382->MON_gyo_bias_y,p382->MON_gyo_bias_z,
                    p382->MON_gyo_sf_z,
                    p382->MON_acc_bias_x,p382->MON_acc_bias_y,p382->MON_acc_bias_z);
            fflush(pf);

            p381.reset();
            p382.reset();
            lastgpssecond = second;
        }
    }
    return flag;
}


int save220msg383_385(FILE *pf, std::shared_ptr< Msg383> p383,
                  std::shared_ptr<Msg385> p385)
{
    int flag = 0;
    if (p383 && p385)
    {
        flag = 1;
    }
    if (flag)
    {
        static double lastgpssecond = 0;
        
        double second = p383->RawGnss_GpsTime;
        if (second > lastgpssecond && fabs(p383->RawGnss_GpsTime-p385->RawGnss_GpsTime)<1e-1)
        {
            fprintf(pf, "%10.4f,%13.9f,%13.9f,%8.4f, %8.4f,%8.4f,%8.4f, %8.4f,%8.4f,%8.4f, %8.4f,%8.4f,%8.4f,%d,%d,%8.4f,%8.4f,%8.4f\n",
                    second,p385->RawGnss_PosLat,p385->RawGnss_PosLon,p385->RawGnss_PosAlt,
                    p385->RawGnss_StdLat,p385->RawGnss_StdLon,p385->RawGnss_StdAlt,
                    p383->RawGnss_VelE,p383->RawGnss_VelN,p383->RawGnss_VelU,
                    p383->RawGnss_StdVelE,p383->RawGnss_StdVelN,p383->RawGnss_StdVelU,
                    p385->RawGnss_PosType, p385->RawGnss_NumSats,
                    p383->RawGnss_Hdop,p383->RawGnss_Vdop,p383->RawGnss_Gdop
                    );
            fflush(pf);

            p383.reset();
            p385.reset();
            lastgpssecond = second;
        }
    }
    return flag;
}
int save220msg388_389(FILE *pf, std::shared_ptr< Msg388> p388,
                  std::shared_ptr<Msg389> p389)
{
    int flag = 0;
    if (p388 && p389)
    {
        flag = 1;
    }
    if (flag)
    {
        static double lastgpssecond = 0;
        
        double second = p388->INS_GpsTime;
        if (second > lastgpssecond && fabs(p388->INS_GpsTime-p389->INS_GpsTime)<1e-1)
        {
            fprintf(pf, "%10.4f,%13.9f,%13.9f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%d,%d\n",
                    second,p388->INS_PosLat,p388->INS_PosLon,p388->INS_PosAlt,
                    p388->INS_StdLat,p388->INS_StdLon,p388->INS_StdAlt,
                    p389->INS_VelE, p389->INS_VelN, p389->INS_VelU,
                    p389->INS_StdVelE,p389->INS_StdVelN,p389->INS_StdVelU,
                    p389->INS_Pitch,p389->INS_Roll,p389->INS_Yaw,
                    p389->INS_StatPos, p389->INS_StatIns
                    );
            fflush(pf);

            p388.reset();
            p389.reset();
            lastgpssecond = second;
        }
    }
    return flag;
}
int IsInter(double time, double EPS_value)
{
    int inttime = round(time);
    int ret = 0;
    if(fabs(time-inttime)<EPS_value)
        ret =1;
    return ret;    
}
int IsEqual(double time1, double time2,double EPS_value)
{
    int ret = 0;
    if(fabs(time1-time2)<EPS_value)
        ret =1;
    return ret;    
}

int save220msgMergeAll(FILE *pf, std::shared_ptr<Msg381>& p381, std::shared_ptr<Msg382>& p382,
                       std::shared_ptr<Msg383>& p383, std::shared_ptr<Msg385>& p385,
                       std::shared_ptr<Msg388>& p388, std::shared_ptr<Msg389>& p389)
{
    if (p381 && p382 && p383 && p385 && p388 && p389)
    {
        double time_381 = p381->MON_GPS_second;
        double time_382 = p382->MON_GPS_second;
        double time_383 = p383->RawGnss_GpsTime;
        double time_385 = p385->RawGnss_GpsTime;
        double time_388 = p388->INS_GpsTime;
        double time_389 = p389->INS_GpsTime;

        double EPS = 1e-3;
        static double lastgpssecond = 0;
        double second = p381->MON_GPS_second;

        if (IsEqual(time_381,time_382,EPS)&&IsEqual(time_381,time_388,EPS)&&IsEqual(time_381,time_389,EPS)&&
            IsInter(time_381, EPS) && IsInter(time_382, EPS)&& IsInter(time_388, EPS) && IsInter(time_389, EPS))

        {
            gtime_t gt0 = gpst2time(p381->MON_TIME_GPS_week, p381->MON_GPS_second);
            gt0 = gpst2utc(gt0);
            fprintf(pf, "%lf,%13.9f,%13.9f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%d,%d, %13.9f,%13.9f,%8.4f,%8.4f,%8.4f,%8.4f,%d,%d, %8.4f,%8.4f,%8.4f, %8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f, %10.7f, %10.7f, %10.7f, %10.7f, %10.7f, %10.7f, %10.7f\n",
                    gt0.time + gt0.sec, p388->INS_PosLat, p388->INS_PosLon, p388->INS_PosAlt,
                    p388->INS_StdLat, p388->INS_StdLon, p388->INS_StdAlt,
                    p389->INS_VelE, p389->INS_VelN, p389->INS_VelU,
                    p389->INS_Roll, p389->INS_Pitch, p389->INS_Yaw,
                    p389->INS_StatPos, p389->INS_StatIns,
                    p385->RawGnss_PosLat, p385->RawGnss_PosLon, p385->RawGnss_PosAlt,
                    p385->RawGnss_StdLat, p385->RawGnss_StdLon, p385->RawGnss_StdAlt,
                    p385->RawGnss_NumSats, p385->RawGnss_PosType,
                    p383->RawGnss_VelE, p383->RawGnss_VelN, p383->RawGnss_VelU,
                    p383->RawGnss_StdVelE, p383->RawGnss_StdVelN, p383->RawGnss_StdVelU,
                    p381->MON_install_imu2car_pitch, p381->MON_install_imu2car_roll,p381->MON_install_imu2car_yaw,
                    p382->MON_gyo_bias_x,p382->MON_gyo_bias_y,p382->MON_gyo_bias_z,
                    p382->MON_acc_bias_x,p382->MON_acc_bias_y,p382->MON_acc_bias_z,
                    p382->MON_gyo_sf_z);
            p381.reset(); p382.reset(); p388.reset(); p389.reset();
        }

    }
}
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

using namespace std;
bool cmp1(string a,string b)///按照字典序从大到小排序
{
    //double a1 = stof(a.find_last_of('/'));
   // double b1 = stof(b);
    // int  aaaa =  a.compare(b)<0;
   string a1 = a.substr(a.find_last_of('/')+1);
   string b1 = b.substr(b.find_last_of('/')+1);
//       string a1 = a.substr(a.find_last_of('/')+1+18);
//    string b1 = b.substr(b.find_last_of('/')+1+18);
   a1 = a1.substr(0,a1.length()-4);
    b1 = b1.substr(0,b1.length()-4);
   double a11 = stod(a1);
    double b11 = stod(b1); 
    int d = a11<b11;
   int c = a1.compare(b1)<0;
    // int  bbbba = strcmp(a.c_str(),b.c_str());
    return d;
}

template <class T, class T2>
T converstd_msgsUInt8MultiArray(std_msgs::UInt8MultiArray::Ptr p220topic, T& pT)
{
    if (p220topic)
    {
        unsigned char buf[512] = {0};
        size_t size = p220topic->data.size();

        if (size > sizeof(buf) / sizeof(buf[0]))
        {
            // 数据太大，无法复制
            throw std::runtime_error("Data too large to fit in buffer.");
        }
        for (unsigned int i = 0; i < p220topic->data.size(); i++)
        {
            buf[i] = p220topic->data.at(i);
        }
        T2 *pg;
        pg = (T2*)(buf);
        T2*pg3 = new T2(*pg);
        pT = T(pg3);        
        int kkk = 1;
        
    }
    return pT;
}
int main(int argc, char *argv[])
{
    // if (argc < 2)
    //     return 1;
    using namespace std;
    char rosbagpathname[1024] = "";
    char outputpathname[1024] = "";
    if (argc == 3)
    {
        strcpy(rosbagpathname, argv[1]);
        strcpy(outputpathname, argv[2]);
    }
    else
    {
        // strcpy(rosbagpathname, "/mnt/share/liujianhe/X8B_X3J/biaoding/0526/0526-2/out/");
        // strcpy(rosbagpathname, "/mnt/logs/04_MG6-002/GND/1201/1201-2/output/rtkimu_hc_220pro/");
        strcpy(rosbagpathname, "/mnt/logs/07_MTD/SL_003/group2/routine11/1228-1/MET/output/rtkimu_hc_220pro/");
        strcpy(outputpathname, "/media/cjq/新加卷/datasets/MemoryDriving/SL-003/routine11/1228-1/rtkimu");
    }

                            //     MaxiDataWheelSpeed maxiDataWheelSpeed;
                            // long long diffTime = ( utc * 1000.0 ) - mLast22BTimestamp;
                            // char diffRrNum = (msgCan1.m22b.RbmWheel_Speed_RR_Pulse - mLast22bMsg.RbmWheel_Speed_RR_Pulse);
                            // if(diffRrNum < -200)
                            // {
                            //     diffRrNum = diffRrNum+255;
                            // }
                            // // printf("BCS_RRWheelSpdEdgesSum:[%d][%d]\n", msgCan1.m22b.BCS_RRWheelSpdEdgesSum, diffRrNum);
                            // diffRrNum = ((diffRrNum) * 10.0 / diffTime + 0.5);
                            // double pulse = (double)(diffRrNum) * 100 / 96 * 10000;
                            // maxiDataWheelSpeed.mTimestamp = utc * 1000.0;
                            // maxiDataWheelSpeed.mWheelSpeed.vector.x = pulse;
                            // maxiDataWheelSpeed.mWheelSpeed.vector.y = 0;
                            // maxiDataWheelSpeed.mWheelSpeed.header.stamp = ros::Time().fromSec(utc);
                            // maxiDataWheelSpeed.mWheelSpeed.header.frame_id = "/blf/wheel/sick/dfs60/right";

    // std::string str("/home/dxw/catkin_ws_LIO_note/data");
    // std::string str("/mnt/share/slam_group/data/2023/0223/");
    string str(rosbagpathname);
    std::vector<std::string> vfilename;
    getFiles(str, vfilename);
    vector<string> vbagfilename;
    map<string, int> vmbagfilename;
    vector<int[10]> vtime;
    vector<int> vtime0;
    int hourhas23 = 0, hourhas00 = 0;
    int IsOneDay = 0, day = 0;
    int readodo = 0;
    for (size_t i = 0; i < vfilename.size(); i++)
    {
        if (readodo == 1)
        {
            if (vfilename[i].find(".bag") != string::npos && vfilename[i].find("2023-") != string::npos)
            {
                vbagfilename.push_back(vfilename[i]);

                // vmbagfilename[i] =vfilename[i];
                char temp[1024] = "";
                GetFileName((char *)vfilename[i].c_str(), temp);
                int ep[10] = {0};
                sscanf(temp, "%4d-%2d-%2d-%2d-%2d-%2d.bag", &ep[0], &ep[1], &ep[2], &ep[3], &ep[4], &ep[5]);
                // vtime.push_back(ep);
                if (ep[3] == 23)
                    hourhas23 = 1;
                if (ep[3] == 0)
                    hourhas00 = 1;
                if (day == 0)
                {
                    day = ep[2];
                }
                else if (day != ep[2])
                {
                    IsOneDay = 1;
                }
                day = ep[2];
            }
        }
        else if (readodo == 0)
        {
            if (vfilename[i].find(".bag") != string::npos /* && vfilename[i].find("2023") != string::npos */)
            {
                vbagfilename.push_back(vfilename[i]);
            }
        }
    }
    // if (readodo == 1)
    //sort(vbagfilename.begin(),vbagfilename.end(),std::greater<int>());
    sort(vbagfilename.begin(),vbagfilename.end(),std::less<std::string>());
    //sort(vbagfilename.begin(),vbagfilename.end(),cmp1);
    for (size_t i = 0; i < vbagfilename.size(); i++)
    {

        if (readodo == 1)
        {
            char temp[1024] = "";
            GetFileName((char *)vbagfilename[i].c_str(), temp);
            int ep[10] = {0};
            sscanf(temp, "%4d-%2d-%2d-%2d-%2d-%2d.bag", &ep[0], &ep[1], &ep[2], &ep[3], &ep[4], &ep[5]);
            int second = ep[2] * 3600 * 24 + ep[3] * 3600 + ep[4] * 60 + ep[5];
            if (hourhas23 * hourhas00 && IsOneDay == 0)
            {
                if (ep[3] == 0)
                {
                    second += 1 * 3600 * 7;
                }
            }
            vmbagfilename[vbagfilename[i]] = second;
        }
        else
        {
            vmbagfilename[vbagfilename[i]] = i;
        }
    }
    vector<pair<string, int>> tVector;
    sortMapByValue(vmbagfilename, tVector);

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "bag_read");

    ros::NodeHandle nh;
    rosbag::Bag rosbag_t;
    string output(outputpathname);
    output = output + "/pointcloud.txt";
    string outputpcd(outputpathname);
    outputpcd = outputpcd + "/pointcloudpcd.txt";
    FILE *pf = fopen(output.c_str(), "w");

    string outputimu(outputpathname);
    outputimu = outputimu + "/Imu.txt";
    FILE *pfimu = fopen(outputimu.c_str(), "w");

    string str2(outputpathname);
    string outputgnss(str2 + "/GnssResult.txt");
    FILE *pfgnss = fopen(outputgnss.c_str(), "w");

    // string str0("/home/dxw/Desktop/jobcode/GnssInsPro2023/Data/里程计/2023-0223-dat/");
    string str0(outputpathname);
    string outputodo(str0 + "/odo.txt");
    FILE *pfodo = fopen(outputodo.c_str(), "w");


    
    outputodo = (string(outputpathname) + "/Gnss381382.txt");
    FILE *pf220_381382 = fopen(outputodo.c_str(), "w");

    outputodo = (string(outputpathname) + "/Gnss38738838938d.txt");
    FILE *pf220_38738838938d = fopen(outputodo.c_str(), "w");

    outputodo = (string(outputpathname) + "/Gnss383385.txt");
    FILE *pf220_383385 = fopen(outputodo.c_str(), "w");

     
    outputodo = (string(outputpathname) + "/GnssInsMerge.txt");
    FILE *pf220_mergeAll = fopen(outputodo.c_str(), "w");



    // string outputpcd(str+"/odoraw.txt");
    // FILE *pf =fopen(output.c_str(),"w");

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    // laserCloudIn = new pcl::PointCloud<PointXYZIRT>[1e5];
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());

    Eigen::Quaterniond q_b_c(0.498141,0.497766, 0.502679, 0.501396);
    Eigen::Vector3d t_bc (1.074, -1.030, 0.828);


    Eigen::Matrix4d Tbc;
    Tbc.setIdentity(4,4);
    Tbc.block<3,3>(0,0)  = q_b_c.toRotationMatrix();
    
    std::cout<< Tbc<<std::endl;
    ROS_DEBUG("fff\n");
    ROS_DEBUG("fff1\n");
    ROS_DEBUG("fff2\n");
    ROS_DEBUG("fff3\n");
    ROS_DEBUG("fff4\n");
    //for (int i = tVector.size() - 1; i >= 0; i--)
    for (int i = 0; i <vbagfilename.size(); i++)
    {
        // if (i==0|| i==3 )continue;
        ROS_INFO("%s\n", vbagfilename[i].c_str());
        // if(i==1)continue;
        //ROS_INFO("%s\n", tVector[i].first.c_str());
        //rosbag_t.open(tVector[i].first, rosbag::bagmode::Read);
        rosbag_t.open(vbagfilename[i], rosbag::bagmode::Read);
        // ROS_INFO("%s\n", vbagfilename[i].c_str());
        sensor_msgs::NavSatFix::Ptr plla[2];
        geometry_msgs::Vector3Stamped::Ptr pvel;
        sensor_msgs::Image::Ptr pImage;

        std::shared_ptr<Msg381>pmsg381 = nullptr;
         std::shared_ptr<Msg382>pmsg382 = nullptr;
         std::shared_ptr<Msg383>pmsg383 = nullptr;
         std::shared_ptr<Msg385>pmsg385 = nullptr;
         std::shared_ptr<Msg388>pmsg388 = nullptr;
         std::shared_ptr<Msg389>pmsg389 = nullptr;
         std::shared_ptr<Msg38d>pmsg38d = nullptr;
        std_msgs::UInt8MultiArray::Ptr p220topic = nullptr;
        for (rosbag::MessageInstance const m : rosbag::View(rosbag_t))
        {
            std::string topic = m.getTopic();
            ros::Time time0 = m.getTime();
            // ROS_INFO("topic=%s, time0=%f", topic.c_str(), time0.toSec());
            double sensor_msg_imu[7] = {0};
            // sensor_msgs::Imu::ConstPtr pp = m.instantiate<sensor_msgs::Imu>();
            // if (pp)
            // {
            //     sensor_msg_imu[0] = pp->angular_velocity.x;
            //     sensor_msg_imu[1] = pp->angular_velocity.y;
            //     sensor_msg_imu[2] = pp->angular_velocity.z;

            //     sensor_msg_imu[3] = pp->linear_acceleration.x;
            //     sensor_msg_imu[4] = pp->linear_acceleration.y;
            //     sensor_msg_imu[5] = pp->linear_acceleration.z;
            //     sensor_msg_imu[6] = time0.toSec();
            // }
            // nav_msgs::Odometry::ConstPtr p2 = m.instantiate<nav_msgs::Odometry>();
            // if (p2)
            // {
            //     p2->pose.pose.position.x;
            // }
            // m.instantiate<sensor_msgs::PointCloud2>();
            if (topic == TOPIC_WHEEL)
            {
                geometry_msgs::Vector3Stamped::ConstPtr p3 = m.instantiate<geometry_msgs::Vector3Stamped>();

                if (p3)
                {
                    int vx = p3->vector.x;
                    int vy = p3->vector.y;

                   

                    // double seco = time0.toSec();

                    // static int iiii = 0;
                    // iiii ++;
                    // if(iiii>567)
                    // {
                    //     int kk = 0;
                    // }
                    // fprintf(pfodo, "%12.3f, %d, %d\n", time0.toSec(), vx, vy);
                    static gtime_t lastgt0 ={0};
                    gtime_t t0;
                    t0.time= (long long)time0.toSec();
                    t0.sec = time0.toSec() -t0.time;
                    t0 = utc2gpst(t0);
                    int week =0;
                    double second = 0;
                    second = time2gpst(t0, &week);
                    if (timediff(t0, lastgt0) > 0)
                    {
                        // fprintf(pfodo, "%d, %.4f, %f\n", week, second, p3->vector.y);
                        fprintf(pfodo, "%d, %.4f, %f\n", week, second, p3->vector.x);
                        // fprintf(pfodo, "%d, %.4f, %d\n", week, second, vx*200);
                        lastgt0 = t0;
                    }
                }
            }
            if (topic == TOPIC_HuaceIMU)
            {
                sensor_msgs::Imu::ConstPtr p4 = m.instantiate<sensor_msgs::Imu>();
                if (p4)
                {
                    double second = p4->header.stamp.toSec();
                    static double lastImusecond = 0;
                    if (second > lastImusecond)
                    {
                        // fprintf(pfimu, "%.8f %9.6f %9.6f %9.6f %9.6f\
                    //  %10.7f %10.7f %10.7f\
                    //   %10.7f %10.7f %10.7f\n",
                        //         second, p4->orientation.w, p4->orientation.x, p4->orientation.y, p4->orientation.z,
                        //         p4->angular_velocity.x, p4->angular_velocity.y, p4->angular_velocity.z,
                        //         p4->linear_acceleration.x, p4->linear_acceleration.y, p4->linear_acceleration.z);
                        // fflush(pfimu);
                        fprintf(pfimu, "%.6f %10.7f %10.7f %10.7f %10.7f %10.7f %10.7f %8.3f\n",
                                second,
                                p4->angular_velocity.x, p4->angular_velocity.y, p4->angular_velocity.z,
                                p4->linear_acceleration.x, p4->linear_acceleration.y, p4->linear_acceleration.z,
                                30/* p4->angular_velocity_covariance[0] */);
                        // fprintf(pfimu, "%.6f %10.7f %10.7f %10.7f %10.7f %10.7f %10.7f %8.3f\n",
                        //         second,
                        //         p4->angular_velocity.y, p4->angular_velocity.x, -p4->angular_velocity.z,
                        //         p4->linear_acceleration.y, p4->linear_acceleration.x, -p4->linear_acceleration.z,
                        //         p4->angular_velocity_covariance[0]);
                        lastImusecond = second;
                        fflush(pfimu);
                    }
                }
            }
            if (topic == TOPIC_HuaceGNSS)
            {
                plla[0] = NULL;
                plla[0] = m.instantiate<sensor_msgs::NavSatFix>();
                plla[1] = plla[0];
                // if (plla[0])
                // {
                //     double second = plla[0]->header.stamp.toSec();

                //     fprintf(pfgnss, "%10.4f %13.9f %13.9f %13.4f  %13.4f  %13.4f  %13.4f \n",
                //             second,plla[0]->latitude,plla[0]->longitude,plla[0]->altitude,
                //             plla[0]->position_covariance[1],plla[0]->position_covariance[2],plla[0]->position_covariance[3]);
                //     fflush(pfgnss);
                // }
            }
            if (topic == TOPIC_HuaceGNSSVEL)
            {
                pvel = nullptr;
                pvel = m.instantiate<geometry_msgs::Vector3Stamped>();
                // if (pvel)
                // {
                //     double second = pvel->header.stamp.toSec();

                //     fprintf(pfgnss, "%10.4f %13.3f %13.3f %13.3f\n",
                //             second,pvel->vector.x,pvel->vector.y,pvel->vector.z);
                //     fflush(pfgnss);
                // }
            }
            if (saveGNSSFILE(pfgnss, plla[0], pvel))
            {
                plla[0] = NULL;
                pvel = NULL;
            }
            if(topic == TOPIC_CAMERA)
            {
                pImage = m.instantiate<sensor_msgs::Image>();
                if (pImage->encoding == sensor_msgs::image_encodings::MONO8)
                {
                    cv_bridge::CvImagePtr cv_ptr;
                    try // 对错误异常进行捕获，检查数据的有效性
                    {
                        cv_ptr = cv_bridge::toCvCopy(pImage, sensor_msgs::image_encodings::MONO8);
                        char base_name[256] ={""};
                      
                        static double lastsecond = 0;
                        double second = pImage->header.stamp.toSec();
                        if(second>lastsecond)
                        {
                                                        // strcpy(base_name, outputpathname);
                            sprintf(base_name, "%s/%f.jpg", outputpathname, second);
                            imwrite(base_name, cv_ptr->image);
                            lastsecond = second;
                        }

                    
                       
                    }
                    catch (cv_bridge::Exception &e) // 异常处理
                    {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        return 1;
                    }
                }
                else if (pImage->encoding == sensor_msgs::image_encodings::BGR8)
                {
                    cv_bridge::CvImagePtr cv_ptr;
                    try // 对错误异常进行捕获，检查数据的有效性
                    {
                        cv_ptr = cv_bridge::toCvCopy(pImage, sensor_msgs::image_encodings::BGR8);
                        char base_name[256] ={""};
                      
                        static double lastsecond = 0;
                        double second = pImage->header.stamp.toSec();
                        if(second>lastsecond)
                        {
                                                        // strcpy(base_name, outputpathname);
                            sprintf(base_name, "%s/%f.jpg", outputpathname, second);
                            imwrite(base_name, cv_ptr->image);
                            lastsecond = second;
                        }

                    
                       
                    }
                    catch (cv_bridge::Exception &e) // 异常处理
                    {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        return 1;
                    }
                }
            }
            if (topic == TOPIC_HC220PRO_381)
            {   
                p220topic = nullptr; 
                p220topic = m.instantiate<std_msgs::UInt8MultiArray>();
                std::shared_ptr<Msg381>pmsg38111  = converstd_msgsUInt8MultiArray<std::shared_ptr<Msg381>, Msg381>(p220topic, pmsg381);
                int kkkd= 0;
            }
            if (topic == TOPIC_HC220PRO_382)
            {
                p220topic = nullptr; 
                p220topic = m.instantiate<std_msgs::UInt8MultiArray>();
               converstd_msgsUInt8MultiArray<std::shared_ptr<Msg382>, Msg382>(p220topic, pmsg382);
                
            }
            if (topic == TOPIC_HC220PRO_383)
            {
                p220topic = nullptr;
                p220topic = m.instantiate<std_msgs::UInt8MultiArray>();
                converstd_msgsUInt8MultiArray<std::shared_ptr<Msg383>, Msg383>(p220topic, pmsg383);
            }
            if (topic == TOPIC_HC220PRO_385)
            {
                p220topic = nullptr;
                p220topic = m.instantiate<std_msgs::UInt8MultiArray>();
                converstd_msgsUInt8MultiArray<std::shared_ptr<Msg385>, Msg385>(p220topic, pmsg385);
            }

            if (topic == TOPIC_HC220PRO_388)
            {
                p220topic = nullptr;
                p220topic = m.instantiate<std_msgs::UInt8MultiArray>();
               converstd_msgsUInt8MultiArray<std::shared_ptr<Msg388>, Msg388>(p220topic, pmsg388);
            }
            if (topic == TOPIC_HC220PRO_389)
            {
                p220topic = nullptr;
                p220topic = m.instantiate<std_msgs::UInt8MultiArray>();
                converstd_msgsUInt8MultiArray<std::shared_ptr<Msg389>, Msg389>(p220topic, pmsg389);
            }
            if (topic == TOPIC_HC220PRO_38d)
            {
                p220topic = nullptr;
                p220topic = m.instantiate<std_msgs::UInt8MultiArray>();
                converstd_msgsUInt8MultiArray<std::shared_ptr<Msg38d>, Msg38d>(p220topic, pmsg38d);
            }

            // save220msg381_382(pf220_381382, pmsg381,pmsg382);
            // save220msg383_385(pf220_383385, pmsg383,pmsg385);
            // save220msg388_389(pf220_38738838938d, pmsg388,pmsg389);

            save220msgMergeAll(pf220_mergeAll, pmsg381,pmsg382, pmsg383,pmsg385,pmsg388, pmsg389);


            

            // if(topic == TOPIC_POINTSCLOUD)
            // {
            //     geometry_msgs::Vector3Stamped::ConstPtr p3 = m.instantiate<geometry_msgs::Vector3Stamped>();

            //     if (p3)
            //     {
            //         // int vx = p3->vector.x;
            //         // int vy = p3->vector.y;

            //         // int kk = 0;

            //         // fprintf(pf,"%12.3f, %d, %d\n",time0.toSec(), vx,vy);
            //     }
            //     sensor_msgs::PointCloud2::ConstPtr p1 = m.instantiate<sensor_msgs::PointCloud2>();
            //     if (p1)
            //     {
            //         double second = p1->header.stamp.toSec();

            //         sensor_msgs::PointCloud2 temp = *p1;
            //         pcl::moveFromROSMsg(temp, *laserCloudIn);

            //         for (size_t i = 0; i < laserCloudIn->points.size(); i++)
            //         {
            //              fprintf(pf,"%.8f %.8f %d %8.3f %8.3f %8.3f \n",second, laserCloudIn->points[i].time,
            //              laserCloudIn->points[i].ring,laserCloudIn->points[i].x,laserCloudIn->points[i].y,laserCloudIn->points[i].z);
            //         }

            //         pcl::io::savePCDFileASCII(outputpcd,*laserCloudIn);
            //         fflush(pf);
            //     }
            // }
            // if(topic == TOPIC_IMU)
            // {
            //     sensor_msgs::Imu::ConstPtr p4 = m.instantiate<sensor_msgs::Imu>();

            //     if(p4)
            //     {
            //         double second = p4->header.stamp.toSec();

            //         fprintf(pfimu,"%.8f %9.6f %9.6f %9.6f %9.6f\
            //          %10.7f %10.7f %10.7f\
            //           %10.7f %10.7f %10.7f\n",second,p4->orientation.w,p4->orientation.x,p4->orientation.y,p4->orientation.z,
            //         p4->angular_velocity.x,p4->angular_velocity.y,p4->angular_velocity.z,
            //         p4->linear_acceleration.x,p4->linear_acceleration.y,p4->linear_acceleration.z
            //         );
            //          fflush(pfimu);
            //     }
            // }
            // if(topic =="ddd")
            // {
            //     nav_msgs::Odometry::ConstPtr p2 =m.instantiate<nav_msgs::Odometry>();
            //     if(p2)
            //     {
            //         double second = p2->header.stamp.toSec();

            //         fprintf(pfodo, "%.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\ 
            //         %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n",
            //         second, p2->pose.pose.position.x,p2->pose.pose.position.y,p2->pose.pose.position.z,
            //         p2->pose.pose.orientation.w,p2->pose.pose.orientation.x,p2->pose.pose.orientation.y,p2->pose.pose.orientation.z,
            //         p2->twist.twist.linear.x,p2->twist.twist.linear.y,p2->twist.twist.linear.z,
            //         p2->twist.twist.angular.x,p2->twist.twist.angular.y,p2->twist.twist.angular.z);
            //          fflush(pfodo);
            //     }
            // }
        }
        rosbag_t.close();
    }

    fclose(pf220_381382);
    fclose(pf220_38738838938d);
    fclose(pf220_383385);
    fclose(pf220_mergeAll);
     fclose(pf);
    fclose(pfodo);
    fclose(pfimu);
    fclose(pfgnss);
}
