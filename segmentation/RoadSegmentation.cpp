//
// Created by garrus on 13.12.18.
//

#include "RoadSegmentation.h"

RoadSegmentation::RoadSegmentation()
{

}

std::pair<OccupancyGrid, TGridMap> RoadSegmentation::calculate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)//, std::shared_ptr<TGridMap>& gridMap)
{
    TGridMap gridMap(cloud, CELL_SIZE);
    fill_grid(gridMap);

    // Calculate the start cell of search
    // Something in front of car
    int row0 = (int)((0 - gridMap.min().x)/CELL_SIZE);
    int col0 = (int)((0 - gridMap.min().y)/CELL_SIZE);
    GridCoord start_coord(row0, col0);

    auto occupancyGrid =  bfs_free_cells_mark(gridMap, start_coord);
    return std::pair(std::move(occupancyGrid), std::move(gridMap));
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
        grid.indexes(row, col).push_back(i);
        grid.z_mean(row, col) += p.z;
        //grid.z_dispersion(row, col) += p.z;
        grid.z_max(row, col) = std::max(grid.z_max(row, col), p.z);
    }

    for(int row = 0; row < grid.rows(); row++)
        for(int col = 0; col < grid.cols(); col++)
            calc_cell(grid, GridCoord(row, col));
}

void RoadSegmentation::calc_cell(TGridMap& grid, const GridCoord& coord)
{
    if(grid.indexes(coord).empty())
    {
        grid.z_max(coord) = 0;
        return;
    }

    // Calculate meand and dispersion
    float z_mean = grid.z_mean(coord) / grid.indexes(coord).size();
    float z_dispersion = 0;

    const auto& indexes = grid.indexes(coord);
    for (int index: indexes)
    {
        float value = grid.cloud_at(index).z - z_mean;
        z_dispersion += value*value;
    }

    grid.z_mean(coord) = z_mean;
    grid.z_dispersion(coord) = z_dispersion / grid.indexes(coord).size();
}

// Mark cells as free using  breadth-first search (BFS)
OccupancyGrid RoadSegmentation::bfs_free_cells_mark(TGridMap& grid, const GridCoord &start_coord)
{
    OccupancyGrid occupancyGrid(grid.rows(), grid.cols(), grid.cell_size());
    std::queue<SearchStep> coords_queue;

    coords_queue.push({start_coord, FORWARD});
    coords_queue.push({start_coord, BACKWARD});

    std::vector<std::vector<bool>> visited(grid.rows(), std::vector<bool>(grid.cols()));

    // Do BFS search from starting coordinate in front, left and right directions
    while(!coords_queue.empty())
    {
        SearchStep step = coords_queue.front();
        GridCoord coord = step.coord;

        if(coord.row < grid.rows() - 1 && !visited[coord.row + 1][coord.col] && step.direction == FORWARD)
            step_next(grid, coords_queue, visited, occupancyGrid, step, GridCoord(coord.row + 1, coord.col));

        if(coord.row > 0               && !visited[coord.row - 1][coord.col] && step.direction == BACKWARD)
            step_next(grid, coords_queue, visited, occupancyGrid, step, GridCoord(coord.row - 1, coord.col));

        if(coord.col > 0               && !visited[coord.row][coord.col - 1])
            step_next(grid, coords_queue, visited, occupancyGrid, step, GridCoord(coord.row,     coord.col - 1));

        if(coord.col < grid.cols() - 1 && !visited[coord.row][coord.col + 1])
            step_next(grid, coords_queue, visited, occupancyGrid, step, GridCoord(coord.row,     coord.col + 1));

        coords_queue.pop();
    }

    return occupancyGrid;
}

// Check next cell and add to the search perepherial if it good
void RoadSegmentation::step_next(const TGridMap& grid, std::queue<SearchStep>& queue, std::vector<std::vector<bool>>& visited, OccupancyGrid& occupancyGrid, SearchStep current, GridCoord next)
{
    bool good = grid.z_dispersion(next) < MAX_DISPERSION; //&& fabs(next_cell.z_mean - current_cell.z_mean) < MAX_Z_DIFF;
    visited[next.row][next.col] = true;

    if(good)
    {
        queue.push({next, current.direction});
        occupancyGrid.at(next) = FREE;
    }
    else
    {
        occupancyGrid.at(next) = OBSTACLE;
    }
}