//
// Created by garrus on 13.12.18.
//

#ifndef ROAD_SEGMENTATION_GRIDMAP_H
#define ROAD_SEGMENTATION_GRIDMAP_H

#include <vector>
#include <pcl/common/common_headers.h>
#include <GridCoord.h>
#include <Grid.h>

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
    GridMap(const typename pcl::PointCloud<TPoint>::Ptr& cloud, float cell_size, TPoint min, TPoint max) :
        CELL_SIZE(cell_size),
        _min(min), _max(max),
        _rows((size_t)ceil((max.x - min.x) / cell_size)), _cols((size_t)ceil((max.y - min.y) / cell_size)),
        _cloud(cloud),
        _z_mean(_rows, _cols),
        _z_max(_rows, _cols),
        _z_dispersion(_rows, _cols),
        _cnt(_rows, _cols),
        _point_cells(cloud->points.size())
    {
        for(int i = 0; i<_rows*_cols; i++)
        {
            _z_mean.data()[i] = 0;
            _z_max.data()[i] = -std::numeric_limits<float>::max();
            _z_dispersion.data()[i] = 0;
            _cnt.data()[i] = 0;
        }
    }

    GridMap(GridMap&& grid) :
        CELL_SIZE(grid.CELL_SIZE),
        _rows(grid._rows), _cols(grid._cols),
        _min(grid._min), _max(grid._max),
        _z_mean(std::move(grid._z_mean)),
        _z_max(std::move(grid._z_max)),
        _z_dispersion(std::move(grid._z_dispersion)),
        _cnt(std::move(grid._cnt)),
        _cloud(grid._cloud),
        _point_cells(std::move(grid._point_cells))
    {
    }

    ~GridMap()
    {
    }


    Grid<float>& z_mean(){ return _z_mean; };
    Grid<float>& z_max(){ return _z_max; };
    Grid<float>& z_dispersion(){ return _z_dispersion; };
    Grid<size_t>& count(){ return _cnt; };

    const Grid<float>& z_mean() const { return _z_mean; };
    const Grid<float>& z_max() const { return _z_max; };
    const Grid<float>& z_dispersion() const { return _z_dispersion; };
    const Grid<size_t>& count() const { return _cnt; };

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

    typename pcl::PointCloud<TPoint>::Ptr _cloud;
    Grid<float> _z_mean;
    Grid<float> _z_max;
    Grid<float> _z_dispersion;
    Grid<size_t> _cnt;
    std::vector<GridCoord> _point_cells;  // coord of cell for each point
    //std::vector<std::vector<int>> _indexes;
};


#endif //ROAD_SEGMENTATION_GRIDMAP_H
