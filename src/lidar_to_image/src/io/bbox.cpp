//
// Created by cjq on 23-9-20.
//
#include "bbox.h"
#include <spdlog/spdlog.h>
#include "file_utils.h"


std::vector<BBox> ReadBBox(std::string path) {
    std::vector<BBox> res;
    std::ifstream ifs(path);
    std::string type;

    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d rpy(0.01208751, 0.0007789972, -1.568911);
    rotation_matrix = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitX());
    Eigen::Vector3d translation(0.76, 0, 2.069604);

    while (ifs >> type) {
        int int_val[3];
        double double_val[12];
        for (int i = 0; i < 3; i++) ifs >> int_val[i];
        for (int i = 0; i < 12; i++) ifs >> double_val[i];
        Eigen::Map<Eigen::Vector3d> hwl(&double_val[4]);  // todo
        Eigen::Map<Eigen::Vector3d> xyz(&double_val[7]);
        double yaw = double_val[10];

        Eigen::Vector3d wlh(hwl[1], hwl[2], hwl[0]);
        // xyz = rotation_matrix.transpose() * (xyz - translation);
        // yaw = yaw - rpy[2] - M_PI / 2;
        yaw = yaw - M_PI / 2;
        res.emplace_back(xyz, wlh, yaw);
        ifs >> std::ws;
    }
    return res;
}


std::map<double, std::vector<BBox>> ReadBBoxes(const std::string &path){
    std::string boxes_path = path;

    std::vector<std::string> file_paths_full;
    std::vector<std::string> file_names;
    getFilesList(file_paths_full, file_names, boxes_path, ".txt", true /*recursive*/);
    std::string path_filter_name;
    if (file_paths_full.empty() && boxes_path.find("/") != boxes_path.npos) {
        path_filter_name = split(boxes_path, "/").back();
        boxes_path = boxes_path.substr(0, boxes_path.rfind("/"));
        getFilesList(file_paths_full, file_names, boxes_path, ".txt", true /*recursive*/);
    }

    std::map<double, std::vector<BBox>> res;
    for (int i = 0; i < file_paths_full.size(); i++) {
        auto file_name = split(file_names[i], ".")[0];
        if (file_name.length() != 23) {
            SPDLOG_WARN("name length not correct! {}", file_name);
            continue;
        }
        if (!path_filter_name.empty()) {
            auto dirs = split(file_paths_full[i], "/");
            if (dirs.size() < 2 ||
                dirs[dirs.size() - 2].find(path_filter_name) == dirs[dirs.size() - 2].npos) {
                continue;
            }
        }
        struct tm t;

        t.tm_year = atoi(file_name.substr(0, 4).c_str()) - 1900;
        t.tm_mon = atoi(file_name.substr(4, 2).c_str()) - 1;
        t.tm_mday = atoi(file_name.substr(6, 2).c_str());
        t.tm_hour = atoi(file_name.substr(9, 2).c_str());
        t.tm_min = atoi(file_name.substr(11, 2).c_str());
        t.tm_sec = atoi(file_name.substr(13, 2).c_str());
        t.tm_isdst = 0;

        time_t t_of_day;
        t_of_day = mktime(&t);
        double timestamp = t_of_day + atoi(file_name.substr(16, 3).c_str()) / 1.e3;
        SPDLOG_INFO("timestamp:{},{}", timestamp, file_name);
        res[timestamp] = ReadBBox(file_paths_full[i]);
    }
    return res;
}

