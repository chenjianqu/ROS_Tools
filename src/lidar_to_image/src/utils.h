//
// Created by cjq on 23-7-25.
//

#ifndef ALIGNTRAJECTORY_UTILS_H
#define ALIGNTRAJECTORY_UTILS_H

#include <iostream>
#include <filesystem>
#include <fstream>
#include <regex>

#include <eigen3/Eigen/Eigen>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <spdlog/spdlog.h>

#include "def.h"
#include "pose_type.h"

using Eigen::Quaterniond;
using Eigen::Matrix3d;

struct ImageInfo{
    string name;
    double timestamp;
    int width;
    int height;
};

std::vector<PoseStamped> ReadPosesFromTxt(const std::string& txt_path);

void WritePosesToTxt(const std::string& txt_path,const std::vector<PoseStamped> &poses);

void WritePosesToTxtComma(const std::string& txt_path,const std::vector<PoseStamped> &poses);


void WriteOnePoseToTxt(const std::string& txt_path,const Posed *pose);

vector<double> StringLineToVector(const string& line);

vector<ImageInfo> ReadImageInfoFromDir(const string& path);

std::optional<PoseStamped> PoseInterpolation(const vector<PoseStamped> &key_poses, double target_time);


template <class T>
inline static std::string PoseToStr(const Pose<T>* pose) {
    auto euler = pose->euler().template cast<double>();
    auto t_d = pose->t();
    return fmt::format("t:{:.3f} {:.3f} {:.3f} euler:{:.3f} {:.3f} {:.3f}",
                       double(t_d.x()),double(t_d.y()),double(t_d.z()),
                       euler.x(),euler.y(),euler.z());
}



#endif //ALIGNTRAJECTORY_UTILS_H
