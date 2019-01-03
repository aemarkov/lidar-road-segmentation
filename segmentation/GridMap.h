//
// Created by garrus on 13.12.18.
//

#ifndef ROAD_SEGMENTATION_GRIDMAP_H
#define ROAD_SEGMENTATION_GRIDMAP_H

#include <vector>
#include <pcl/common/common_headers.h>
#include <GridCoord.h>
#include <Grid.h>

enum Obstacle
{
    UNKNOW = 0,
    FREE,
    OBSTACLE
};


/**
 * \brief 2D Occupancy Grid with some additional information
 *
 *  - Store 2D occupancy grid
 *  - Store original point cloud
 *  - Store 2D grid coordinate of each point in point cloud
 *    (e.g. for visualization purposes)
 *  - Store size, cell size, min/max of bounding box
 *
 * @tparam TPoint pcl::PointCloud template argument
 */
template <class TPoint>
class GridMap
{
public:

    /**
     * \brief Initialize the empty grid map of given PointCloud
     *
     * Calculate the required grid size based on cell size and point cloud size.
     * Do not perform placing points into the cells.
     *
     * @param cloud
     * @param cell_size
     */
    GridMap(const typename pcl::PointCloud<TPoint>::ConstPtr& cloud, float cell_size, TPoint min, TPoint max) :
        CELL_SIZE(cell_size),
        _min(min), _max(max),
        _rows((size_t)ceil((max.x - min.x) / cell_size)), _cols((size_t)ceil((max.y - min.y) / cell_size)),
        _cloud(cloud),
        _point_cells(cloud->points.size()),
        _obstacles(_rows, _cols)
    {
    }

    GridMap(GridMap&& grid) :
        CELL_SIZE(grid.CELL_SIZE),
        _rows(grid._rows), _cols(grid._cols),
        _min(grid._min), _max(grid._max),
        _cloud(grid._cloud),
        _point_cells(std::move(grid._point_cells)),
        _obstacles(std::move(grid._obstacles))
    {
    }

    ~GridMap()
    {
    }


    Grid<Obstacle>& obstacles() { return _obstacles; }
    const Grid<Obstacle>& obstacles() const { return _obstacles; }

    TPoint& cloud_at(size_t index) { return _cloud->at(index); }
    const TPoint& cloud_at(size_t index) const { return _cloud->at(index); }
    typename pcl::PointCloud<TPoint>::Ptr cloud(){ return _cloud; }
    size_t cloud_size() const {return _cloud->points.size(); }

    GridCoord& point_cell(size_t index) { return _point_cells[index]; }
    GridCoord point_cell(size_t index) const { return _point_cells[index]; }

    const TPoint& min() const { return _min; }
    const TPoint& max() const { return _max; }

    size_t rows() const { return _rows; }
    size_t cols() const { return _cols; }
    GridCoord size() const { return GridCoord(_rows, _cols); }

    float cell_size() const { return CELL_SIZE; }

private:
    const float CELL_SIZE;
    const TPoint _min, _max;
    const size_t _rows, _cols;

    typename pcl::PointCloud<TPoint>::ConstPtr _cloud;
    std::vector<GridCoord> _point_cells;              // coord of cell for each point
    Grid<Obstacle> _obstacles;
};


#endif //ROAD_SEGMENTATION_GRIDMAP_H
