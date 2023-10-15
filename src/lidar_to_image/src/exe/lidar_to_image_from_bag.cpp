//
// Created by cjq on 23-9-20.
//
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <filesystem>

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <gflags/gflags.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>

#include <calib_manager/calib_manager.h>

#include "pose_type.h"
#include "utils.h"
#include "cloud_utils.h"
#include "bbox_remover.h"

using namespace std;
namespace fs=std::filesystem;


DEFINE_string(bag_path,"","bag_path");
DEFINE_string(write_path,"./depth","bag_path");
DEFINE_string(poses_path,"","bag_path");
DEFINE_string(obj_det_path,"","obj_det_path");
DEFINE_string(image_path,"","image_path");

DEFINE_string(lidar_topic,"/lidar/hs/pandar64/topmiddle","lidar_topic");

DEFINE_string(calib_date, "20230718", "calib_date");
DEFINE_string(calib_vehicle, "A12-001", "calib_vehicle");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    if(FLAGS_bag_path.empty()){
        SPDLOG_ERROR("-bag_path is missing");
        return -1;
    }
    if(FLAGS_poses_path.empty()){
        SPDLOG_ERROR("-poses_path is missing");
        return -1;
    }
    if(FLAGS_obj_det_path.empty()){
        SPDLOG_ERROR("-obj_det_path is missing");
        return -1;
    }
    if(FLAGS_image_path.empty()){
        SPDLOG_ERROR("-image_path is missing");
        return -1;
    }
    if(!fs::exists(FLAGS_bag_path)){
        SPDLOG_ERROR("{} not exists!",FLAGS_bag_path);
        return -1;
    }
    if(!fs::exists(FLAGS_write_path)){
        SPDLOG_WARN("{} not exists!",FLAGS_write_path);
        fs::create_directory(FLAGS_write_path);
    }

    std::string time_str = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
    ros::init(argc, argv, "lidar_to_image_from_bag_"+time_str);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::shared_ptr<calib_manager::CalibManager> calib;
    try {
        calib = std::make_shared<calib_manager::CalibManager>(FLAGS_calib_vehicle,std::stoi(FLAGS_calib_date));
    }
    catch (std::exception& e){
        SPDLOG_ERROR("在构造CalibManager时报错，信息：{}",e.what());
        std::terminate();
    }
    auto graph = calib->GetSensorGraph();

    ///外参读取
    Posed T_bl,T_lb;
    string calib_imu_idx="imu-1";
    string calib_lidar_idx="lidar-1";
    string calib_cam_idx="camera-1";
    if (graph.count(calib_lidar_idx) > 0 && graph[calib_lidar_idx].count(calib_imu_idx) > 0) {
        Eigen::Matrix4d mat_lidar2imu = graph[calib_lidar_idx][calib_imu_idx];
        T_bl = Posed(mat_lidar2imu);
        T_lb = T_bl.Inverse();
    } else {
        SPDLOG_ERROR("Read calib_manager GetSensorGraph({},{}) failed!", calib_lidar_idx, calib_imu_idx);
        std::terminate();
    }
    SPDLOG_INFO("T_bl: {} from calib_manager", PoseToStr(&T_bl));

    Posed T_cl,T_lc;
    if (graph.count(calib_cam_idx) > 0 && graph[calib_cam_idx].count(calib_lidar_idx) > 0) {
        Eigen::Matrix4d mat_cam2lidar = graph[calib_cam_idx][calib_lidar_idx];
        T_lc = Posed(mat_cam2lidar);
        T_cl = T_lc.Inverse();
    } else {
        SPDLOG_ERROR("Read calib_manager GetSensorGraph({},{}) failed!", calib_cam_idx, calib_lidar_idx);
        std::terminate();
    }
    SPDLOG_INFO("T_cl: {} from calib_manager", PoseToStr(&T_cl));

    ///内参读取
    const auto& cameras = calib->GetCameras();
    calib_manager::Camera cam= cameras.at(calib_cam_idx);
    Mat3d K = cam.intrinsic;

    ///去畸变图像的内参
    Mat3d K_distorted = Mat3d::Identity();
    K_distorted(0,0) = 1463.1887235131180;
    K_distorted(1,1) = 1462.9860405817967;
    K_distorted(0,2) = 1759.5;
    K_distorted(1,2) = 539.5;


    ///位姿读取
    vector<PoseStamped> poses_imu = ReadPosesFromTxt(FLAGS_poses_path);
    vector<PoseStamped> poses_lidar;
    poses_lidar.reserve(poses_imu.size());
    for(auto& T_wb:poses_imu){
        PoseStamped T_wl = T_wb * T_bl;
        poses_lidar.push_back(T_wl);
    }

    ///读取图像信息
    vector<ImageInfo> image_infos = ReadImageInfoFromDir(FLAGS_image_path);

    ///用于动态物体去除
    BBoxRemover bbox_remover(FLAGS_obj_det_path);

    pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("viewer"));

    cout<<"Ready to read..."<<endl;
    rosbag::Bag bag;
    bag.open(FLAGS_bag_path, rosbag::bagmode::Read); //打开一个bag文件
    if(!bag.isOpen()){
        SPDLOG_ERROR("Open {} failed!",FLAGS_bag_path);
        return -1;
    }

    std::vector<std::string> topics; //设置需要遍历的topic
    topics.emplace_back(FLAGS_lidar_topic);

    bool is_first_run = true;
    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下

    for(auto it=view.begin();it!=view.end();++it){
        if(!ros::ok()){
            break;
        }
        auto topic=it->getTopic();
        if(topic == FLAGS_lidar_topic){
            sensor_msgs::PointCloud2ConstPtr cloud_msg = it->instantiate<sensor_msgs::PointCloud2>();
            if(is_first_run){
                is_first_run = false;
                for (auto& field:cloud_msg->fields) {
                    cout<<fmt::format("name:{} offset:{} datatype:{} count:{}",
                                      field.name,field.offset,field.datatype,field.count)<<endl;
                }
            }

            double ts = cloud_msg->header.stamp.toSec();
            PointCloud::Ptr pcl_cloud(new PointCloud);
            pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
            cout<<fmt::format("cloud:{:.6f} size:{}",ts,pcl_cloud->size())<<endl;

            ///剔除动态物体的点云
            //bool ok = bbox_remover.RemovePoints(ts,pcl_cloud);
            //if(!ok){
            //    SPDLOG_ERROR("bbox_remover.RemovePoints() failed!");
            //}

            ///位姿插值
            auto curr_pose = PoseInterpolation(poses_lidar,ts);
            if(!curr_pose){
                SPDLOG_WARN("PoseInterpolation in {} failed!",ts);
                continue;
            }

            ///去畸变，得到位于ts时刻下的点云
            PointCloud::Ptr un_cloud = PointCloudDistortion(poses_lidar,*pcl_cloud,ts);
            if(!un_cloud){
                SPDLOG_WARN("PointCloudDistortion() failed!");
                continue;
            }

            ///将点云变换到最近的图像坐标系
            auto [cam_cloud,curr_info]  = CloudTransformToImage(image_infos,poses_lidar,
                                                              un_cloud,*curr_pose,T_cl);
            if(!cam_cloud){
                SPDLOG_WARN("CloudTransformToImage() failed!");
                continue;
            }

            ///将点投到图像上
            cv::Mat depth_image = ProjectToImage(curr_info,cam_cloud,K);

            /*
             src.convertTo(dst, type, scale, shift)
                缩放并转换到另外一种数据类型：
                dst：目的矩阵；
                type：需要的输出矩阵类型，或者更明确的，是输出矩阵的深度，如果是负值（常用-1）则输出矩阵和输入矩阵类型相同；
                scale:比例因子；
                shift：将输入数组元素按比例缩放后添加的值；
             */
            //cv::Mat depth_vis;
            //depth_image.convertTo(depth_vis,CV_8UC1,255./(150-1.),0);

            cv::Mat image_rec = cv::imread((fs::path(FLAGS_image_path)/curr_info.name).string());
            cv::imwrite((fs::path(FLAGS_write_path)/curr_info.name).string(),depth_image);

            cv::Mat img_vis = ProjectToImage(cam_cloud,K_distorted,image_rec);
            cv::resize(img_vis,img_vis,cv::Size(),0.5,0.5);
            cv::imshow("depth",img_vis);
            cv::waitKey(10);

            auto un_cloud_xyzi = MaxieyeToXYZI(cam_cloud);
            viewer->removeAllPointClouds();  // 移除当前所有点云
            viewer->addPointCloud<pcl::PointXYZI>(un_cloud_xyzi, "viewer");
            viewer->updatePointCloud<pcl::PointXYZI>(un_cloud_xyzi, "viewer");
            viewer->spinOnce(1);
        }
    }
    bag.close();

    return 0;
}
