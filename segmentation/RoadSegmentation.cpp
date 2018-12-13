//
// Created by garrus on 13.12.18.
//

#include "RoadSegmentation.h"

RoadSegmentation::RoadSegmentation()
{

}

TGridMap RoadSegmentation::calculate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    TGridMap grid(cloud, CELL_SIZE);
    fill_grid(grid);

    // Calculate the start cell of search
    // Something in front of car
    // TODO: make it better
    int row0 = (int)((0 - grid.min().x)/CELL_SIZE);
    int col0 = (int)((0 - grid.min().y)/CELL_SIZE);
    GridCoord start_coord(row0, col0);

    bfs_free_cells_mark(grid, start_coord);

    return grid;
}

void RoadSegmentation::set_params(float cell_size, float max_dispersion, float max_z_diff)
{
    CELL_SIZE = cell_size;
    MAX_DISPERSION =  max_dispersion;
    MAX_Z_DIFF = max_z_diff;
}

// Place every point to the corresponding grid cell
// calculate z-mean
void RoadSegmentation::fill_grid(TGridMap &grid)
{
    for(int i = 0; i<grid.cloud_size(); i++)
    {
        const pcl::PointXYZRGB p = grid.cloud_at(i);
        int row = (int)((p.x - grid.min().x)/CELL_SIZE);
        int col = (int)((p.y - grid.min().y)/CELL_SIZE);
        grid.at(row, col).indexes.push_back(i);
        grid.at(row, col).z_mean += p.z;
    }
}

// Mark cells as free using  breadth-first search (BFS)
void RoadSegmentation::bfs_free_cells_mark(TGridMap& grid, const GridCoord &start_coord)
{
    std::queue<GridCoord> coords_queue;
    calc_cell(grid, start_coord);
    coords_queue.push(start_coord);

    std::cout << "Size: " << grid.rows() << " " << grid.cols() << std::endl;

    // Do BFS search from starting coordinate in front, left and right directions
    while(!coords_queue.empty())
    {
        GridCoord coord = coords_queue.front();

        if(coord.row < grid.rows() - 1 && !grid.at(coord.row + 1, coord.col).visited)
        {
            step_next(grid, coords_queue, coord, GridCoord(coord.row + 1, coord.col));
            std::cout << coord.row + 1 << " " << coord.col << std::endl;
        }
        if(coord.col > 0               && !grid.at(coord.row, coord.col - 1).visited)
            step_next(grid, coords_queue, coord, GridCoord(coord.row,     coord.col - 1));
        if(coord.col < grid.cols() - 1 && !grid.at(coord.row, coord.col + 1).visited)
            step_next(grid, coords_queue, coord, GridCoord(coord.row,     coord.col + 1));
        coords_queue.pop();

    }
}

// Check next cell and add to the search perepherial if it good
void RoadSegmentation::step_next(TGridMap& grid, std::queue<GridCoord>& queue, GridCoord current, GridCoord next)
{
    Cell& current_cell = grid.at(current);
    Cell& next_cell = grid.at(next);

    calc_cell(grid, next);
    bool good = next_cell.z_dispersion < MAX_DISPERSION; //&& fabs(next_cell.z_mean - current_cell.z_mean) < MAX_Z_DIFF;
    next_cell.visited = true;

    Color color;
    if(good)
    {
        color = Color(0, 255, 0);
        queue.push(next);
    }
    else
        color = Color(0, 0, 255);

    if(!next_cell.indexes.empty())
        dbg_color_cell(grid, next, color);
}

void RoadSegmentation::calc_cell(TGridMap& grid, const GridCoord& coord)
{
    Cell& cell = grid.at(coord);
    if(cell.indexes.empty())
        return;

    // Calculate meand and dispersion
    cell.z_mean /= cell.indexes.size();
    for (int index: cell.indexes)
    {
        float value = grid.cloud_at(index).z - cell.z_mean;
        cell.z_dispersion += value * value;
    }
    cell.z_dispersion /= cell.indexes.size();
}

// Color cells in point cloud and 2D picture in debug purpose
void RoadSegmentation::dbg_color_cell(TGridMap& grid, GridCoord coord, Color color)
{
    grid.image().at<Color>(coord.row, coord.col) = color;

    for(auto index: grid.at(coord).indexes)
    {
        grid.cloud_at(index).r = color[2];
        grid.cloud_at(index).g = color[1];
        grid.cloud_at(index).b = color[0];
    }
}