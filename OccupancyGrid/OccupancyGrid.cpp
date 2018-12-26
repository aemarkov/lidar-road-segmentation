//
// Created by garrus on 16.12.18.
//

#include "OccupancyGrid.h"
#include <iostream>

OccupancyGrid::OccupancyGrid(size_t rows, size_t cols, float cell_size) :
    _rows(rows), _cols(cols),
    CELL_SIZE(cell_size),
    _grid(rows, cols)
{
}

OccupancyGrid::OccupancyGrid(OccupancyGrid &&grid) :
    _grid(std::move(grid._grid))
{
}


Grid<Obstacle>& OccupancyGrid::grid() { return _grid; }
const Grid<Obstacle>&  OccupancyGrid::grid() const { return _grid; }

size_t OccupancyGrid::rows() const { return _rows; }
size_t OccupancyGrid::cols() const { return _cols; }
float OccupancyGrid::cell_size() const { return CELL_SIZE; }
