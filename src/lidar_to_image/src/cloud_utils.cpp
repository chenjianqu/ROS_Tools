//
// Created by cjq on 23-9-20.
//
#include <sensor_msgs/PointCloud2.h>

#include "cloud_utils.h"
#include "pose_type.h"
#include "utils.h"



PointCloud::Ptr PointCloudDistortion(const vector<PoseStamped> &poses_lidar,
                                     PointCloud& cloud,double end_time){
    double start_time = cloud.points[0].timestamp; //lidar帧的开始时刻
    double interval = end_time - start_time;

    auto poses0 = PoseInterpolation(poses_lidar,start_time);
    auto poses1 = PoseInterpolation(poses_lidar, end_time);
    if(!poses0 || !poses1){
        return {};
    }
    PoseStamped T_wl0 = *poses0;
    PoseStamped T_wl1 = *poses1;
    PoseStamped T_l1_l0 = T_wl1.Inverse() * T_wl0;//用于将点从 l0坐标系 变换到 l1坐标系

    PointCloud::Ptr cloud_distorted(new PointCloud);

    for(const auto& point:cloud.points){
        if(has_nan(point))
            continue;
        if (point.timestamp < start_time || point.timestamp > end_time)
            continue;
        double s = (point.timestamp - start_time) / interval;

        Eigen::Quaterniond q_l1_li = Eigen::Quaterniond::Identity().slerp(s, T_l1_l0.q()).normalized();
        const Eigen::Vector3d t_l1_li = s * T_l1_l0.t();

        Vec3d point_l1 = q_l1_li*Eigen::Vector3d(point.x, point.y, point.z) + t_l1_li;

        PointT un_point = point;
        un_point.timestamp = end_time;
        un_point.x = point_l1.x();
        un_point.y = point_l1.y();
        un_point.z = point_l1.z();

        cloud_distorted->push_back(un_point);
    }

    return cloud_distorted;
}


std::tuple<PointCloud::Ptr,ImageInfo> CloudTransformToImage(const vector<ImageInfo> &image_infos,
                                      const vector<PoseStamped> &poses_lidar,
                                      const PointCloud::Ptr& cloud,
                                      const PoseStamped& curr_pose,
                                      const Posed& T_cl
                                      ){
    ///找到最近的图像
    //在数组的[begin, end)区间中二分查找第一个大于等于value的数，找到返回该数字的地址，没找到则返回end。
    ImageInfo key;
    key.timestamp = curr_pose.timestamp();
    auto it = std::lower_bound(image_infos.begin(),image_infos.end(),key,
                               [](const auto& element,const auto& key){
        return element.timestamp < key.timestamp;
    });
    if(it==image_infos.end()){
        SPDLOG_WARN("In CloudTransformToImage(),can no found image timestamp for {}",key.timestamp);
        return {PointCloud::Ptr(),{}};
    }
    else if(it == image_infos.begin()){
        SPDLOG_WARN("In CloudTransformToImage(),{} is not in image range:[]",key.timestamp,
                    image_infos[0].timestamp,image_infos[image_infos.size()-1].timestamp);
        return {PointCloud::Ptr(),{}};
    }

    const auto& candidate0 = *std::prev(it,1);
    const auto& candidate1 = *it;
    double image_timestamp;
    if(std::abs(curr_pose.timestamp()-candidate0.timestamp) < std::abs(curr_pose.timestamp()-candidate1.timestamp)){
        image_timestamp = candidate0.timestamp;
    }
    else{
        image_timestamp = candidate1.timestamp;
    }

    ///得到图像的位姿
    auto pose_ptr = PoseInterpolation(poses_lidar,image_timestamp);
    if(!pose_ptr){
        SPDLOG_ERROR("PoseInterpolation() failed!");
        return {PointCloud::Ptr(),candidate1};
    }
    PoseStamped T_w_lc = *pose_ptr;

    ///位姿变换
    PoseStamped T_w_l0 = curr_pose;
    PoseStamped T_lc_l0 = T_w_lc.Inverse() * T_w_l0;

    PointCloud::Ptr cloud_warped(new PointCloud);
    cloud_warped->points.reserve(cloud->size());

    for(const auto& point:cloud->points){
        auto point_wrap = point;
        Vec3d P_lc = T_lc_l0 * Vec3d(point.x,point.y,point.z);
        Vec3d P_c = T_cl * P_lc;

        point_wrap.x = P_c.x();
        point_wrap.y = P_c.y();
        point_wrap.z = P_c.z();
        cloud_warped->push_back(point_wrap);
    }

    return {cloud_warped,candidate1};
}


cv::Mat ProjectToImage(ImageInfo& image_info,PointCloud::Ptr& cloud,const Mat3d& K){
    cv::Mat img(image_info.height,image_info.width,CV_32FC1,cv::Scalar(0));

    for(auto& point:cloud->points){
        Vec3d xyd = K*Vec3d(point.x,point.y,point.z);
        float depth = xyd.z();
        if(depth<=0)
            continue;
        cv::Point2i xy(int(xyd.x()/depth),int(xyd.y()/depth));
        if(xy.x >=0 && xy.x < image_info.width &&
           xy.y >=0 && xy.y < image_info.height ){
            img.at<float>(xy) = depth;
        }
    }
    return img;
}


cv::Mat ProjectToImage(PointCloud::Ptr& cloud,const Mat3d& K,const cv::Mat &image){
    cv::Mat out_image = image.clone();

    for(auto& point:cloud->points){
        Vec3d xyd = K*Vec3d(point.x,point.y,point.z);
        float depth = xyd.z();
        if(depth<=0)
            continue;
        cv::Point2i xy(int(xyd.x()/depth),int(xyd.y()/depth));
        if(xy.x >=0 && xy.x < out_image.cols &&
           xy.y >=0 && xy.y < out_image.rows ){
            int value = int(depth / 50. * 255.) % 255;
            cv::circle(out_image,xy,2,cv::Scalar(value,255,255-value),-1);
        }
    }
    return out_image;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr MaxieyeToXYZI(PointCloud::Ptr &cloud){
    using PointXYZI = pcl::PointXYZI;
    using PointCloud2 = pcl::PointCloud<PointXYZI>;
    PointCloud2::Ptr cloud2 (new PointCloud2);
    cloud2->points.reserve(cloud->size());

    for(auto& point:cloud->points){
        PointXYZI point_new;
        point_new.x = point.x;
        point_new.y = point.y;
        point_new.z = point.z;
        point_new.intensity = point.intensity;
        cloud2->push_back(point_new);
    }
    return cloud2;
}
