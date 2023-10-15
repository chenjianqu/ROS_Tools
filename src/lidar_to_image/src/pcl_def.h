//
// Created by cjq on 23-9-20.
//

#ifndef LIDAR_TO_IMAGE_PCL_TYPES_H
#define LIDAR_TO_IMAGE_PCL_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "def.h"


// rslidar和velodyne的格式有微小的区别
// rslidar的点云格式
struct RsPointXYZIRT_f {
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    std::uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT_f,(float, x, x)(float, y, y)(float, z, z)(std::uint8_t,
        intensity,intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))


// maxieye的点云格式
struct MaxieyePointXYZIRT {
    double x;
    double y;
    double z;
    float intensity;
    std::uint16_t ring;
    double timestamp;
    bool filter = false;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(MaxieyePointXYZIRT,
(double, x, x)(double, y, y)(double, z, z)(float, intensity,
intensity)(
std::uint16_t, ring, ring)(double, timestamp, timestamp))


using PointT = RsPointXYZIRT_f;
using PointCloud = pcl::PointCloud<PointT>;


#endif //LIDAR_TO_IMAGE_PCL_TYPES_H
