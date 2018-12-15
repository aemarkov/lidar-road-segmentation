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
    /*int row0 = (int)((0 - grid.min().x)/CELL_SIZE);
    int col0 = (int)((0 - grid.min().y)/CELL_SIZE);
    GridCoord start_coord(row0, col0);

    bfs_free_cells_mark(grid, start_coord);*/

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
        Cell& cell = grid.at(row, col);
        cell.indexes.push_back(i);
        cell.z_mean += p.z;
        cell.z_max = std::max(cell.z_max, p.z);
    }

    for(int row = 0; row < grid.rows(); row++)
        for(int col = 0; col < grid.cols(); col++)
            calc_cell(grid, GridCoord(row, col));
}

void RoadSegmentation::calc_cell(TGridMap& grid, const GridCoord& coord)
{
    Cell& cell = grid.at(coord);
    if(cell.indexes.empty())
    {
        cell.z_max = 0;
        return;
    }

    // Calculate meand and dispersion
    cell.z_mean /= cell.indexes.size();
    for (int index: cell.indexes)
    {
        float value = grid.cloud_at(index).z - cell.z_mean;
        cell.z_dispersion += value * value;
    }
    cell.z_dispersion /= cell.indexes.size();
}

void RoadSegmentation::average_cells_neighbours(TGridMap& grid, const GridCoord& coord, int radius)
{
    int r_start = std::max(coord.row - radius, 0);
    int r_end = std::min(coord.row + radius, (int)grid.rows());
    int c_start = std::max(coord.col - radius, 0);
    int c_end = std::min(coord.col + radius, (int)grid.cols());

    float z_mean = 0;
    float z_dispersion = 0;
    int cnt = 0;

    for(int row = r_start; row < r_end; row++)
    {
        for(int col = c_start; col < c_end; col++)
        {
            z_mean += grid.at(row, col).z_mean;
            z_dispersion += grid.at(row, col).z_dispersion;
            cnt++;
        }
    }

    grid.at(coord).z_mean = z_mean/cnt;
    grid.at(coord).z_dispersion = z_dispersion/cnt;
}

// Mark cells as free using  breadth-first search (BFS)
void RoadSegmentation::bfs_free_cells_mark(TGridMap& grid, const GridCoord &start_coord)
{
    std::queue<SearchStep> coords_queue;
    calc_cell(grid, start_coord);
    coords_queue.push({start_coord, FORWARD});
    coords_queue.push({start_coord, BACKWARD});

    // Do BFS search from starting coordinate in front, left and right directions
    while(!coords_queue.empty())
    {
        SearchStep step = coords_queue.front();
        GridCoord coord = step.coord;

        if(coord.row < grid.rows() - 1 && !grid.at(coord.row + 1, coord.col).visited && step.direction == FORWARD)
            step_next(grid, coords_queue, step, GridCoord(coord.row + 1, coord.col));
        if(coord.row > 0               && !grid.at(coord.row - 1, coord.col).visited && step.direction == BACKWARD)
            step_next(grid, coords_queue, step, GridCoord(coord.row - 1, coord.col));
        if(coord.col > 0               && !grid.at(coord.row, coord.col - 1).visited)
            step_next(grid, coords_queue, step, GridCoord(coord.row,     coord.col - 1));
        if(coord.col < grid.cols() - 1 && !grid.at(coord.row, coord.col + 1).visited)
            step_next(grid, coords_queue, step, GridCoord(coord.row,     coord.col + 1));
        coords_queue.pop();
    }
}

// Check next cell and add to the search perepherial if it good
void RoadSegmentation::step_next(TGridMap& grid, std::queue<SearchStep>& queue, SearchStep current, GridCoord next)
{
    Cell& current_cell = grid.at(current.coord);
    Cell& next_cell = grid.at(next);

    bool good = next_cell.z_dispersion < MAX_DISPERSION; //&& fabs(next_cell.z_mean - current_cell.z_mean) < MAX_Z_DIFF;
    next_cell.visited = true;

    if(good)
    {
        next_cell.obstacle = FREE;
        queue.push({next, current.direction});
    }
    else
    {
        next_cell.obstacle = OBSTACLE;
    }
}