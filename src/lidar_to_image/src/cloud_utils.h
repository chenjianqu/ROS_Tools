//
// Created by cjq on 23-9-20.
//

#ifndef LIDAR_TO_IMAGE_CLOUD_UTILS_H
#define LIDAR_TO_IMAGE_CLOUD_UTILS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "def.h"
#include "pcl_def.h"
#include "pose_type.h"
#include "utils.h"

template <typename T>
inline static bool has_nan(T point) {
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        return true;
    } else {
        return false;
    }
}

/**
 * 将激光点云去畸变
 * @param poses_lidar 所有关键帧的位姿
 * @param cloud 原始的点云
 * @param end_time 最后一个点的时刻，作为激光的时间戳
 * @return
 */
PointCloud::Ptr PointCloudDistortion(const vector<PoseStamped> &poses_lidar,
                                     PointCloud& cloud,
                                     double end_time);

pcl::PointCloud<pcl::PointXYZI>::Ptr MaxieyeToXYZI(PointCloud::Ptr &cloud);


std::tuple<PointCloud::Ptr,ImageInfo>
CloudTransformToImage(const vector<ImageInfo> &image_infos,
                      const vector<PoseStamped> &poses_lidar,
                      const PointCloud::Ptr& cloud,
                      const PoseStamped& curr_pose,
                      const Posed& T_cl
);

cv::Mat ProjectToImage(ImageInfo& image_info,PointCloud::Ptr& cloud,const Mat3d& K);

cv::Mat ProjectToImage(PointCloud::Ptr& cloud,const Mat3d& K,const cv::Mat &image);



#endif //LIDAR_TO_IMAGE_CLOUD_UTILS_H
