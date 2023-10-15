//
// Created by cjq on 23-9-20.
//

#include "file_utils.h"
#include <dirent.h>
#include <spdlog/spdlog.h>


void getFilesList(std::vector<std::string> &file_path_full, std::vector<std::string> &file_name,
                  std::string dirpath, std::string suffix, bool recursive) {
    file_path_full.clear();
    file_name.clear();
    DIR *dir = opendir(dirpath.c_str());
    if (dir == NULL) {
        SPDLOG_ERROR("opendir error:{}", dirpath);
        return;
    } else {
        SPDLOG_INFO("opendir ===> {}", dirpath);
    }

    std::vector<std::string> allPath;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {  // It's dir
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
            std::string dirNew = dirpath + "/" + entry->d_name;
            std::vector<std::string> temp_path;
            std::vector<std::string> temp_filename;
            getFilesList(temp_path, temp_filename, dirNew, suffix, recursive);
            file_path_full.insert(file_path_full.end(), temp_path.begin(), temp_path.end());
            file_name.insert(file_name.end(), temp_filename.begin(), temp_filename.end());

        } else {
            std::string name = entry->d_name;
            if (name.find(suffix) != name.size() - suffix.size()) continue;
            std::string full_path = dirpath + "/" + name;
            file_path_full.push_back(full_path);
            file_name.push_back(name);
        }
    }
    closedir(dir);
}

