//
// Created by garrus on 13.12.18.
//

#ifndef ROAD_SEGMENTATION_GRIDMAP_H
#define ROAD_SEGMENTATION_GRIDMAP_H

#include <vector>
#include <pcl/common/common_headers.h>
#include <opencv2/opencv.hpp>


/**
 * Integer two-dimension coordinate in grid
 */
struct GridCoord
{
    int row, col;
    GridCoord(){}
    GridCoord(int row, int col)
    {
        this->row = row;
        this->col = col;
    }
};

/**
 * 2D-grid representation of Point Cloud
 * @tparam TPoint pcl::PointCloud template argument
 * @tparam TCell  type of the cells in grid
 */
template <class TPoint, class TCell>
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
    GridMap(const typename pcl::PointCloud<TPoint>::Ptr& cloud, float cell_size) :
        CELL_SIZE(cell_size)
    {
        // Get point cloud size
        pcl::getMinMax3D(*cloud, _min, _max);
        float x_size = _max.x - _min.x;
        float y_size = _max.y - _min.y;

        // Get grid size and create array for grid
        _rows = (size_t)ceil(x_size / cell_size);
        _cols = (size_t)ceil(y_size / cell_size);
        _cloud = cloud;

        _cells = new TCell[_rows*_cols];
    }

    ~GridMap()
    {
        delete[] _cells;
        _cells = nullptr;
    }

    TCell& at(size_t row, size_t col) { return _cells[row * _cols + col]; }
    TCell& at(const GridCoord& coord) {  return _cells[coord.row * _cols + coord.col]; }
    const TCell& at(size_t row, size_t col) const { return _cells[row * _cols + col]; }
    const TCell& at(const GridCoord& coord) const { return _cells[coord.row * _cols + coord.col]; }

    TPoint& cloud_at(size_t index) { return _cloud->at(index); }
    const TPoint& cloud_at(size_t index) const { return _cloud->at(index); }

    const TPoint& min() const { return _min; }
    const TPoint& max() const { return _max; }

    size_t rows() const { return _rows; }
    size_t cols() const { return _cols; }

    typename pcl::PointCloud<TPoint>::Ptr cloud(){ return _cloud; }
    typename pcl::PointCloud<TPoint>::ConstPtr cloud() const{ return _cloud; }
    size_t cloud_size() const {return _cloud->points.size(); }

    float cell_size() const { return CELL_SIZE; }



private:
    const float CELL_SIZE;

    typename pcl::PointCloud<TPoint>::Ptr _cloud;
    TPoint _min, _max;
    size_t _rows, _cols;
    TCell* _cells;
};


#endif //ROAD_SEGMENTATION_GRIDMAP_H
