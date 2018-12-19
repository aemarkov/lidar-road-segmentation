//
// Created by garrus on 16.12.18.
//

#ifndef ROAD_SEGMENTATION_GRIDCOORD_H
#define ROAD_SEGMENTATION_GRIDCOORD_H

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

#endif //ROAD_SEGMENTATION_GRIDCOORD_H
