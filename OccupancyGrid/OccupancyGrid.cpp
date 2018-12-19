//
// Created by garrus on 16.12.18.
//

#include "OccupancyGrid.h"
#include <iostream>

OccupancyGrid::OccupancyGrid(size_t rows, size_t cols, float cell_size)
{
    //std::cout << "OccupancyGrid()\n";
    _rows = rows;
    _cols = cols;
    CELL_SIZE = cell_size;
    _grid = new Obstacle[_rows * _cols];
}

OccupancyGrid::OccupancyGrid(OccupancyGrid &&grid)
{
    //std::cout << "OccupancyGrid(OccupancyGrid&&)\n";
    _grid = grid._grid;
    _rows = grid._rows;
    _cols = grid._cols;
    CELL_SIZE = grid.CELL_SIZE;
    grid._grid = nullptr;
}


OccupancyGrid::~OccupancyGrid()
{
    //std::cout << "~OccupancyGrid()\n";
    if(_grid!=nullptr)
    {
        delete[] _grid;
        _grid = nullptr;
    }
}

Obstacle &OccupancyGrid::at(size_t row, size_t col) { return _grid[row * _cols + col]; }
const Obstacle &OccupancyGrid::at(size_t row, size_t col) const { return _grid[row * _cols + col]; }
Obstacle &OccupancyGrid::at(GridCoord coord) { return _grid[coord.row * _cols + coord.col]; }
const Obstacle &OccupancyGrid::at(GridCoord coord) const { return _grid[coord.row * _cols + coord.col]; }

size_t OccupancyGrid::rows() const { return _rows; }
size_t OccupancyGrid::cols() const { return _cols; }
float OccupancyGrid::cell_size() const { return CELL_SIZE; }
