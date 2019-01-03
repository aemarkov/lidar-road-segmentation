//
// Created by garrus on 13.12.18.
//

#ifndef ROAD_SEGMENTATION_ROADSEGMENTATION_H
#define ROAD_SEGMENTATION_ROADSEGMENTATION_H

#include <pcl/common/common_headers.h>
#include <boost/make_shared.hpp>
#include <cassert>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "GridMap.h"

using TGridMap = GridMap<pcl::PointXYZ>;

class RoadSegmentation
{
public:
    RoadSegmentation(float cell_size, float distance_treshold);
    ~RoadSegmentation();
    TGridMap calculate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
private:

    const float CELL_SIZE;
    const float DISTANCE_THRESHOLD;
    pcl::SACSegmentation<pcl::PointXYZ> seg;

};

#endif //ROAD_SEGMENTATION_ROADSEGMENTATION_H
