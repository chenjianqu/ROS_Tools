
#include "bbox_remover.h"
#include <spdlog/spdlog.h>

BBoxRemover::BBoxRemover(const std::string& bbox_path) {
    Reset(bbox_path);
}

void BBoxRemover::Reset(const std::string& bbox_path) {
    if (!bbox_path.empty()) {
        bbox_map_ = ReadBBoxes(bbox_path);
        SPDLOG_INFO("bbox_map.size():{} begin:{}", bbox_map_.size(), bbox_map_.begin()->first);
    } else {
        bbox_map_.clear();
        SPDLOG_WARN("bbox_path is empty!");
    }
}

bool BBoxRemover::RemovePoints(double ts_lidar, pcl::PointCloud<RsPointXYZIRT_f>::Ptr &cloud) const {
    const double tolerate = 0.01;
    auto cur_bboxes = bbox_map_.lower_bound(ts_lidar - tolerate);
    if (cur_bboxes != bbox_map_.end()) {
        SPDLOG_INFO("obj time_diff:{}", fabs(cur_bboxes->first - ts_lidar));
    } else {
        return false;
    }
    bool bboxMatched = (cur_bboxes != bbox_map_.end() && fabs(cur_bboxes->first - ts_lidar) < tolerate);
    if (bboxMatched) {
        RemovePointsInBBox(cloud, cur_bboxes->second);
        return true;
    }
    return false;
}



/** \brief RemovePointsInBBox
 * \param[in] cloud: lidar cloud need to be undistorted
 * \param[in] dRlc: delta rotation
 * \param[in] dtlc: delta displacement
 */
void RemovePointsInBBox(PointCloud::Ptr &cloud, const std::vector<BBox>& bboxes) {
    int cur_idx = 0;
    for (int i = 0; i < cloud->size(); i++) {
        bool in_bbox = false;
        for (auto bbox: bboxes) {
            if (bbox.isIn(cloud->points[i].getVector3fMap().cast<double>())) {
                in_bbox = true;
                break;
            }
        }
        if (!in_bbox) {
            cloud->points[cur_idx++] = cloud->points[i];
        }
    }
    cloud->points.resize(cur_idx);
}
