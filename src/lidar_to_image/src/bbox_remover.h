#pragma once

#include "def.h"
#include "pcl_def.h"
#include "io/bbox.h"

class BBoxRemover {
public:
    explicit BBoxRemover(const std::string& bbox_path);

    void Reset(const std::string& bbox_path);

    bool RemovePoints(double ts_lidar, PointCloud::Ptr &cloud) const;

private:
    std::map<double, std::vector<BBox>> bbox_map_;
};

void RemovePointsInBBox(PointCloud::Ptr &cloud, const std::vector<BBox>& bboxes);
