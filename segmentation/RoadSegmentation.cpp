//
// Created by garrus on 13.12.18.
//

#include "RoadSegmentation.h"

RoadSegmentation::RoadSegmentation()
{

}

RoadSegmentation::~RoadSegmentation()
{
    if(_kernel!=nullptr)
        delete[] _kernel;
}

std::pair<OccupancyGrid, TGridMap> RoadSegmentation::calculate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    auto gridMap = create_grid(cloud);
    //interpolate(gridMap);

    // Calculate the start cell of search
    // Something in front of car
    int row0 = (int)((0 - gridMap.min().x)/CELL_SIZE);
    int col0 = (int)((0 - gridMap.min().y)/CELL_SIZE);
    GridCoord start_coord(row0, col0);

    auto occupancyGrid =  bfs_free_cells_mark(gridMap, start_coord);
    //OccupancyGrid occupancyGrid(gridMap.rows(), gridMap.cols(), gridMap.cell_size());
    return std::pair(std::move(occupancyGrid), std::move(gridMap));
}

void RoadSegmentation::set_params(float cell_size, float max_dispersion, float max_z_diff, GridCoord kernel_size, int iterations)
{
    CELL_SIZE = cell_size;
    MAX_DISPERSION =  max_dispersion;
    MAX_Z_DIFF = max_z_diff;
    ITERATIONS = iterations;

    if(KERNEL_SIZE.row != kernel_size.row || KERNEL_SIZE.col != kernel_size.col)
    {
        if(_kernel!=nullptr)
            delete[] _kernel;

        KERNEL_SIZE = kernel_size;
        create_linar_kernel(_kernel, KERNEL_SIZE);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Place every point to the corresponding grid cell
// calculate z-mean
TGridMap RoadSegmentation::create_grid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*cloud, min, max);
    TGridMap grid(cloud, CELL_SIZE, min, max);

    for(int i = 0; i<grid.cloud_size(); i++)
    {
        const pcl::PointXYZRGB p = grid.cloud_at(i);
        int row = (int)((p.x - grid.min().x)/CELL_SIZE);
        int col = (int)((p.y - grid.min().y)/CELL_SIZE);
        grid.count().at(row, col)++;
        grid.z_mean().at(row, col) += p.z;
        grid.point_cell(i) = GridCoord(row, col);
        grid.z_max().at(row, col) = std::max(grid.z_max().at(row, col), p.z);
        grid.z_dispersion().at(row, col) += p.z*p.z;
    }

    for(int row = 0; row < grid.rows(); row++)
        for(int col = 0; col < grid.cols(); col++)
            calc_cell(grid, GridCoord(row, col));

    return grid;
}

void RoadSegmentation::calc_cell(TGridMap& grid, const GridCoord& coord)
{
    if(grid.count().at(coord) == 0)
    {
        grid.z_max().at(coord) = 0; // otherwise float - numeric_limits<float>::max
        return;
    }

    // Calculate meand and dispersion
    size_t count = grid.count().at(coord);
    float z_mean = grid.z_mean().at(coord) / count;
    grid.z_mean().at(coord) = z_mean;
    grid.z_dispersion().at(coord) = grid.z_dispersion().at(coord)/count - z_mean*z_mean;
}

// Fill holes (interpolate using convolution)
void RoadSegmentation::interpolate(TGridMap& grid)
{
    float* buffer = new float[grid.rows()*grid.cols()];
    for(int i = 0; i<grid.rows()*grid.cols(); i++)
        buffer[i] = 0;

    GridCoord size = grid.size();
    interpolate_array(grid.z_mean().data(), buffer, size, _kernel, KERNEL_SIZE, ITERATIONS, grid.count());
    interpolate_array(grid.z_max().data(), buffer, size, _kernel, KERNEL_SIZE, ITERATIONS, grid.count());
    interpolate_array(grid.z_dispersion().data(), buffer, size, _kernel, KERNEL_SIZE, ITERATIONS, grid.count());
    delete[] buffer;

}

void RoadSegmentation::interpolate_array(float*& src, float*& buffer, GridCoord size, float* kernel, GridCoord kernel_size, int iterations, const Grid<size_t>& count)
{
    //TODO: zero-padding или что-то такое
    // Когда я так буферы свапаю, у меня может будет буфером-приемником старый буфер данных.
    // И в нем на краях старые значеия, которые сверткой не перезаписываются. Но с ними идет свертка.
    // Так что это может исказить значения

    for(int i = 0; i<iterations; i++)
    {
        convolution(src, buffer, size, kernel, kernel_size, count);
        std::swap(src, buffer);
    }
}

void RoadSegmentation::convolution(float* src, float* dst, GridCoord size, float* kernel, GridCoord kernel_size, const Grid<size_t>& count)
{
    assert(kernel_size.row % 2 != 0 && kernel_size.col % 2 != 0);

    size_t r_cnt = size.row - kernel_size.row + 1;
    size_t c_cnt = size.col - kernel_size.col + 1;
    uint8_t half_width = kernel_size.row/2;
    uint8_t half_height = kernel_size.col/2;

    for(size_t row = 0; row<r_cnt; row++)
    {
        for(size_t col = 0; col<c_cnt; col++)
        {
            float res = 0;
            if(count.at(row + half_width, col + half_height)==0)
            {
                for (size_t k_row = 0; k_row < kernel_size.row; k_row++)
                {
                    for (size_t k_col = 0; k_col < kernel_size.col; k_col++)
                    {
                        float k = kernel[k_row * kernel_size.col + k_col];
                        res += src[(row + k_row) * size.col + col + k_col] * k;
                    }
                }
            }
            else
                res = src[(row + half_width) * size.col + (col + half_height)];

            dst[(row + half_width) * size.col + (col + half_height)] = res;
        }
    }
}

void RoadSegmentation::create_linar_kernel(float*& kernel, GridCoord size)
{
    /* Linear kernel
     * High value in center, zeros in corners
     *
     * k[x,y] = k*(R - r)
     * R - kernel radius sqrt((rows/2)^2 + (cols/2)^2)
     * r - dist from (x,y) to kernel center
     * k = 1/R
     *
     * Then normalize
     */

    /*kernel = new float[9] {
            0, 0, 0,
            0, 1, 0,
            0, 0, 0
    };

    return;*/

    assert(size.row % 2 !=0 && size.col % 2 != 0);
    int r_row = size.row/2;
    int r_col = size.col/2;
    float r = sqrt(r_row*r_row + r_col*r_col);
    float k = 1/r;
    float sum = 0;  // for normalization

    kernel = new float[size.row * size.col];
    for(int row = 0; row<size.row; row++)
    {
        for(int col = 0; col<size.col; col++)
        {
            float wtf = sqrt((row - r_row)*(row - r_row) + (col - r_col)*(col - r_col));
            float val = k*(r - wtf);
            kernel[row*size.col+col] = val;
            sum+=val;
        }
    }

    // Normalization
    for(int i = 0; i<size.row*size.col; i++)
        kernel[i]/=sum;
}

// Mark cells as free using  breadth-first search (BFS)
OccupancyGrid RoadSegmentation::bfs_free_cells_mark(TGridMap& grid, const GridCoord &start_coord)
{
    OccupancyGrid occupancyGrid(grid.rows(), grid.cols(), grid.cell_size());
    std::queue<SearchStep> coords_queue;

    // TODO: Do something with this consts
    float dx1 = 5.5 / CELL_SIZE;
    float dx2 = 6.5 / CELL_SIZE;
    coords_queue.push({GridCoord(start_coord.row+dx1, start_coord.col), FORWARD});
    coords_queue.push({GridCoord(start_coord.row-dx2, start_coord.col), BACKWARD});

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
    bool good = grid.z_dispersion().at(next) < MAX_DISPERSION;
    //bool good = fabs(grid.z_max().at(next)- grid.z_max().at(current.coord)) < MAX_Z_DIFF;
    visited[next.row][next.col] = true;

    if(good)
    {
        queue.push({next, current.direction});
        occupancyGrid.grid().at(next) = FREE;
    }
    else
    {
        occupancyGrid.grid().at(next) = OBSTACLE;
    }
}

