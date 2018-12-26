//
// Created by garrus on 16.12.18.
//

#ifndef ROAD_SEGMENTATION_OCCUPANCYGRID_H
#define ROAD_SEGMENTATION_OCCUPANCYGRID_H

#include <GridCoord.h>
#include <Grid.h>
#include <cstdio>

enum Obstacle
{
    UNKNOW = 0,
    FREE,
    OBSTACLE
};

/**
 * Two dimensional grid represents free and obstacle space
 */
class OccupancyGrid
{
public:
    OccupancyGrid(size_t rows, size_t cols, float cell_size);
    OccupancyGrid(OccupancyGrid&& grid);

    Grid<Obstacle>& grid();
    const Grid<Obstacle>& grid() const;

    size_t rows() const;
    size_t cols() const;
    float cell_size() const;

//private:
    size_t _rows, _cols;
    float CELL_SIZE;
    Grid<Obstacle> _grid;
};


#endif //ROAD_SEGMENTATION_OCCUPANCYGRID_H
