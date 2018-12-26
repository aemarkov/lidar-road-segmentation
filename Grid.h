//
// Created by garrus on 20.12.18.
//

#ifndef ROAD_SEGMENTATION_ARRAY2D_H
#define ROAD_SEGMENTATION_ARRAY2D_H

#include <cstdlib>
#include <GridCoord.h>

/**
 * Simple grid (two dimensional array)
 * @tparam T
 */
template <class T>
class Grid
{
public:
    Grid(){}
    Grid(size_t rows, size_t cols)
    {
        _array = new T[rows*cols];
        _rows = rows;
        _cols = cols;
    }

    Grid(Grid&& grid)
    {
        _array = grid._array;
        _rows = grid._rows;
        _cols = grid._cols;
        grid._array = nullptr;
    }

    ~Grid()
    {
        if(_array!= nullptr)
        {
            delete[] _array;
            _array = nullptr;
        }
    }

    T& at(size_t row, size_t col) {  return _array[row * _cols + col]; }
    const T& at(size_t row, size_t col) const { return _array[row * _cols + col]; }
    T& at(GridCoord coord) { return _array[coord.row * _cols + coord.col]; }
    const T& at(GridCoord coord) const { return _array[coord.row * _cols + coord.col]; }

    T*& data() { return _array; }

    size_t rows() const { return _rows; }
    size_t cols() const { return _cols; }
    GridCoord size() const { return GridCoord(_rows, _cols); }

private:
    T* _array = nullptr;
    size_t _rows, _cols;
};

#endif //ROAD_SEGMENTATION_ARRAY2D_H
