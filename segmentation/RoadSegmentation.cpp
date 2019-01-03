//
// Created by garrus on 13.12.18.
//

#include "RoadSegmentation.h"

RoadSegmentation::RoadSegmentation(float cell_size, float distance_treshold) :
    CELL_SIZE(cell_size),
    DISTANCE_THRESHOLD(distance_treshold)
{
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(DISTANCE_THRESHOLD);
}

RoadSegmentation::~RoadSegmentation()
{
}

TGridMap RoadSegmentation::calculate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // RANSAC plane estimation
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    // Find cloud size and create occupancy grid
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*cloud, min, max);
    TGridMap grid(cloud, CELL_SIZE, min, max);


    std::vector<bool> invert_indexes(cloud->points.size(), true);
    for(int i = 0; i<inliers.indices.size(); i++)
        invert_indexes[inliers.indices[i]] = false;

    std::cout << grid.rows() << " " << grid.cols() << std::endl;

    // Fill occupancy grid with obstacles
    for(int i = 0; i<grid.cloud_size(); i++)
    {
        if(invert_indexes[i])
        {
            const auto &point = cloud->at(i);
            size_t row = (size_t) ((point.x - min.x) / CELL_SIZE);
            size_t col = (size_t) ((point.y - min.y) / CELL_SIZE);
            grid.obstacles().at(row, col) = OBSTACLE;
            grid.point_cell(i) = GridCoord(row, col);
        }
    }

    return grid;
}
