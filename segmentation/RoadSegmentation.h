//
// Created by garrus on 13.12.18.
//

#ifndef ROAD_SEGMENTATION_ROADSEGMENTATION_H
#define ROAD_SEGMENTATION_ROADSEGMENTATION_H

#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/make_shared.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <algorithm>
#include <optional>

#include "GridMap.h"

enum Obstacle
{
    UNKNOW = 0,
    FREE,
    OBSTACLE
};

struct Cell
{
    std::vector<int> indexes;
    float z_mean = 0;
    float z_dispersion = 0;
    float z_max = -std::numeric_limits<float>::max();
    bool visited = false;
    Obstacle obstacle;

};

enum Direction
{
    FORWARD, BACKWARD
};

struct SearchStep
{
    GridCoord coord;
    Direction  direction;
};

using TGridMap = GridMap<pcl::PointXYZRGB, Cell>;
using Color = cv::Vec3b;

class RoadSegmentation
{
public:
    RoadSegmentation();

    TGridMap calculate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    void set_params(float cell_size, float max_dispersion, float max_z_diff);

    TGridMap &grid();

private:

    float CELL_SIZE;
    float MAX_DISPERSION;
    float MAX_Z_DIFF;

    // Place every point to the corresponding grid cell
    // calculate z-mean
    void fill_grid(TGridMap &cloud_info);

    // Calculate cell's stastistics
    void calc_cell(TGridMap &grid, const GridCoord &coord);

    void average_cells_neighbours(TGridMap &grid, const GridCoord &coord, int radius);

    // Mark cells as free using  breadth-first search (BFS)
    void bfs_free_cells_mark(TGridMap &grid, const GridCoord &start_coord);

    // Check next cell and add to the search perepherial if it good
    void step_next(TGridMap &grid, std::queue<SearchStep> &queue, SearchStep current, GridCoord next);
};

#endif //ROAD_SEGMENTATION_ROADSEGMENTATION_H
