#pragma once

#include <cmath>
#include <fstream>
#include <utility>
#include "def.h"


class BBox {
public:
    BBox(Eigen::Vector3d _xyz, Eigen::Vector3d _wlh, double _yaw)
    : xyz(std::move(_xyz)), wlh(std::move(_wlh)), yaw(_yaw) {
        yaw_mat << std::cos(yaw), std::sin(yaw), -std::sin(yaw), std::cos(yaw);
    }

    bool isIn(const Eigen::Vector3d &point) {
        Eigen::Vector3d point_diff;
        point_diff.head<2>() = yaw_mat.transpose() * (point.head<2>() - xyz.head<2>());
        point_diff[2] = point[2] - xyz[2];
        // if (point_diff.norm()<3) return true;
        if ((point_diff.array().abs() > wlh.array()).any())
            return false;
        return true;
    }

private:
    Eigen::Vector3d xyz;
    Eigen::Vector3d wlh;  // todo
    double yaw;
    Eigen::Matrix2d yaw_mat;
};


std::vector<BBox> ReadBBox(std::string path);

std::map<double, std::vector<BBox>> ReadBBoxes(const std::string &path);

