//
// Created by garrus on 13.12.18.
//

#ifndef ROAD_SEGMENTATION_GRIDMAP_H
#define ROAD_SEGMENTATION_GRIDMAP_H

#include <vector>
#include <pcl/common/common_headers.h>
#include <GridCoord.h>

/**
 * \brief 2D-grid representation of Point Cloud
 *
 * Every cell contains statistical information about inner points
 *  - max z
 *  - mean z
 *  - z dispersion
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
    GridMap(const typename pcl::PointCloud<TPoint>::Ptr& cloud, float cell_size) :
        CELL_SIZE(cell_size)
    {
        //std::cout << "GridMap()\n";

        // Get point cloud size
        pcl::getMinMax3D(*cloud, _min, _max);
        float x_size = _max.x - _min.x;
        float y_size = _max.y - _min.y;

        // Get grid size and create array for grid
        _rows = (size_t)ceil(x_size / cell_size);
        _cols = (size_t)ceil(y_size / cell_size);
        _cloud = cloud;

        _z_mean = new float[_rows*_cols];
        _z_max = new float[_rows*_cols];
        _z_dispersion = new float[_rows*_cols];
        _indexes = std::vector<std::vector<int>>(_rows*_cols);

        for(int i = 0; i<_rows*_cols; i++)
        {
            _z_mean[i] = 0;
            _z_max[i] = -std::numeric_limits<float>::max();
            _z_dispersion[i] = 0;
        }
    }

    GridMap(GridMap&& grid) :
        CELL_SIZE(grid.CELL_SIZE)
    {
        //std::cout << "GridMap(&&)\n";

        _z_mean = grid._z_mean;
        _z_max = grid._z_max;
        _z_dispersion = grid._z_dispersion;
        _cloud = grid._cloud;
        _indexes = std::move(grid.indexes());
        _rows = grid._rows;
        _cols = grid._cols;
        _min = grid._min;
        _max = grid._max;

        grid._z_mean = nullptr;
        grid._z_max = nullptr;
        grid._z_dispersion = nullptr;
    }

    ~GridMap()
    {
        //std::cout << "~GridMap()\n";

        if(_z_mean != nullptr)
        {
            delete[] _z_mean;
            delete[] _z_max;
            delete[] _z_dispersion;
            _z_mean = nullptr;
            _z_max = nullptr;
            _z_dispersion = nullptr;
        }
    }

    float& z_mean(size_t row, size_t col) {  return _z_mean[row * _cols + col]; }
    float& z_max(size_t row, size_t col) {  return _z_max[row * _cols + col]; }
    float& z_dispersion(size_t row, size_t col) {  return _z_dispersion[row * _cols + col]; }
    std::vector<int>& indexes(size_t row, size_t col) { return _indexes[row * _cols + col]; }
    const float z_mean(size_t row, size_t col) const {  return _z_mean[row * _cols + col]; }
    const float z_max(size_t row, size_t col) const {  return _z_max[row * _cols + col]; }
    const float z_dispersion(size_t row, size_t col) const {  return _z_dispersion[row * _cols + col]; }
    const std::vector<int>& indexes(size_t row, size_t col) const { return _indexes[row * _cols + col]; }

    float& z_mean(GridCoord coord) {  return _z_mean[coord.row * _cols + coord.col]; }
    float& z_max(GridCoord coord) {  return _z_max[coord.row * _cols + coord.col]; }
    float& z_dispersion(GridCoord coord) {  return _z_dispersion[coord.row * _cols + coord.col]; }
    std::vector<int>& indexes(GridCoord coord) { return _indexes[coord.row * _cols + coord.col]; }
    const float z_mean(GridCoord coord) const {  return _z_mean[coord.row * _cols + coord.col]; }
    const float z_max(GridCoord coord) const {  return _z_max[coord.row * _cols + coord.col]; }
    const float z_dispersion(GridCoord coord) const {  return _z_dispersion[coord.row * _cols + coord.col]; }
    const std::vector<int>& indexes(GridCoord coord) const { return _indexes[coord.row * _cols + coord.col]; }

    float*& z_mean() { return _z_mean; }
    float*& z_max() { return _z_max; }
    float*& z_dispersion() { return _z_dispersion; }
    std::vector<std::vector<int>>& indexes() { return _indexes; };

    TPoint& cloud_at(size_t index) { return _cloud->at(index); }
    const TPoint& cloud_at(size_t index) const { return _cloud->at(index); }
    typename pcl::PointCloud<TPoint>::Ptr cloud(){ return _cloud; }
    size_t cloud_size() const {return _cloud->points.size(); }

    const TPoint& min() const { return _min; }
    const TPoint& max() const { return _max; }

    size_t rows() const { return _rows; }
    size_t cols() const { return _cols; }
    GridCoord size() const { return GridCoord(_rows, _cols); }

    float cell_size() const { return CELL_SIZE; }



private:
    const float CELL_SIZE;

    typename pcl::PointCloud<TPoint>::Ptr _cloud;
    TPoint _min, _max;
    size_t _rows, _cols;

    float* _z_mean;
    float* _z_max;
    float* _z_dispersion;
    std::vector<std::vector<int>> _indexes;
};


#endif //ROAD_SEGMENTATION_GRIDMAP_H
