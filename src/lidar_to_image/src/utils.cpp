//
// Created by cjq on 23-7-25.
//

#include "utils.h"

namespace fs=std::filesystem;


std::vector<PoseStamped> ReadPosesFromTxt(const std::string& txt_path){
    if(!fs::exists(txt_path)){
        return {};
    }

    std::vector<PoseStamped> output;
    std::ifstream infile(txt_path);

    std::string line;
    while(std::getline(infile,line)){
        vector<double> token = StringLineToVector(line);
        PoseStamped pose;
        pose.timestamp() = token[0];
        pose.t().x() = token[1];
        pose.t().y() = token[2];
        pose.t().z() = token[3];
        pose.q().x() = token[4];
        pose.q().y() = token[5];
        pose.q().z() = token[6];
        pose.q().w() = token[7];
        output.push_back(pose);
    }
    return output;
}



void WritePosesToTxt(const std::string& txt_path,const std::vector<PoseStamped> &poses){
    std::ofstream fout(txt_path);
    for(auto &pose:poses){
        fout<<fmt::format(
                "{} {} {} {} {} {} {} {}",
                pose.timestamp(), pose.t().x(), pose.t().y(), pose.t().z(),
                pose.q().x(), pose.q().y(), pose.q().z(), pose.q().w()) <<std::endl;
    }
    fout.close();
}


void WritePosesToTxtComma(const std::string& txt_path,const std::vector<PoseStamped> &poses){
    std::ofstream fout(txt_path);
    for(auto &pose:poses){
        fout<<fmt::format(
                "{},{},{},{},{},{},{},{}",
                pose.timestamp(), pose.t().x(), pose.t().y(), pose.t().z(),
                pose.q().x(), pose.q().y(), pose.q().z(), pose.q().w()) <<std::endl;
    }
    fout.close();
}


void WriteOnePoseToTxt(const std::string& txt_path,const Posed *pose){
    std::ofstream fout(txt_path);
    fout<<fmt::format(
            "{} {} {} {} {} {} {} {}",
            0,pose->t().x(),pose->t().y(),pose->t().z(),
            pose->q().x(),pose->q().y(),pose->q().z(),pose->q().w()) <<std::endl;
    fout.close();

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


vector<ImageInfo> ReadImageInfoFromDir(const string& path){
    if(!fs::is_directory(path)){
        SPDLOG_ERROR("{} is not dir!",path);
        return {};
    }
    vector<ImageInfo> infos;
    for (auto const& dir_entry : std::filesystem::directory_iterator{path}){
        string extension = dir_entry.path().extension().string();
        if(extension==".png" || extension==".jpg"){
            cv::Mat img = cv::imread(dir_entry.path().string());
            ImageInfo info;
            info.name = dir_entry.path().filename().string();
            info.timestamp = std::stod(dir_entry.path().filename().stem().string());
            info.height = img.rows;
            info.width = img.cols;
            infos.push_back(info);
        }
    }

    std::sort(infos.begin(),infos.end(),[](const auto& a,const auto& b){
        return a.timestamp < b.timestamp;
    });

    if(infos.empty()){
        SPDLOG_ERROR("In ReadImageInfoFromDir(),dir:{} is empty()",path);
    }

    return infos;
}




/**
* 位姿插值
* @param key_poses 关键帧的位姿
* @param sample_time 要插值的时间点
* @return 插值后的位姿
*/
std::optional<PoseStamped> PoseInterpolation(const vector<PoseStamped> &key_poses, double target_time) {
    CHECK_GE(key_poses.size(), 2) << "In PoseInterpolation(),key_poses.size() can not less 2";
    PoseStamped output;

    int k1 = 0, k2 = 1;

    //位姿t在k1关键帧的左边,使用k1,k2插值到左边
    if (target_time < key_poses[k1].timestamp()) {
        SPDLOG_WARN("In PoseInterpolation(),t < k1 and k2, t:{:.3f},k1:{:.3f},未实现",
                    target_time, key_poses[k1].timestamp());
        return std::nullopt;
    }
    //位姿t在k2关键帧的右边，此时应向右查找关键帧
    else if (target_time > key_poses[k2].timestamp()) {
        if (k2 >= key_poses.size()) {//如果无法向右查找，则使用当前的两个kp进行插值
            SPDLOG_WARN("In PoseInterpolation(),t > k1 and k2, t:{:.3f},k2:{:.3f},未实现",
                        target_time, key_poses[k1].timestamp());
            return std::nullopt;
        } else { //向右查找
            while (k2 < key_poses.size() && target_time > key_poses[k2].timestamp()) {
                k1++;
                k2++;
            }
        }
    }

    ///时刻刚好相等的情况下
    if (target_time == key_poses[k1].timestamp()) {
        return key_poses[k1];
    }
    else if (target_time == key_poses[k2].timestamp()) {
        return key_poses[k2];
    }
    //位姿t在k1,k2的中间
    else if (target_time > key_poses[k1].timestamp() && target_time < key_poses[k2].timestamp()) {
        double percent = (target_time - key_poses[k1].timestamp()) / (key_poses[k2].timestamp() - key_poses[k1].timestamp());
        Quaterniond Q_t = key_poses[k1].q().slerp(percent, key_poses[k2].q());//球面插值
        Q_t.normalize();

        output.timestamp() = target_time;
        output.q() = Q_t;
        output.t() = key_poses[k1].t() + percent * (key_poses[k2].t() - key_poses[k1].t());

    }
    else { //待插值时刻在插值区间的右边
        SPDLOG_WARN("In PoseInterpolation(),待插值时刻t:{}在插值区间的右边,未实现",target_time);
        return std::nullopt;
    }

    return output;
}




