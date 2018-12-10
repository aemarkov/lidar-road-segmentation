#include <iostream>
#include <string>
#include <experimental/filesystem>
#include <optional>
#include <queue>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/make_shared.hpp>
#include <opencv2/opencv.hpp>
#include <omp.h>

#include "kitti/kitti.h"

using namespace std;
namespace fs = std::experimental::filesystem;

const fs::path kitti_dir = "/media/garrus/Seagate Expansion Drive/KITTI/Road/data_road";
const fs::path training_dir = "training";
const fs::path testing_dir = "testing";
const fs::path calib_dir = "calib";
const fs::path velodyne_dir = "velodyne";
const fs::path rgb_left_dir = "image_2";
const fs::path gt_dir = "gt_image_2";

struct Cell
{
    std::vector<int> indexes;
    float z_mean = 0;
    float z_dispersion = 0;
    bool visited = false;
};

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

// Create KITTI filename of category, type, number and extension
// e.g. um_lane_0000000.bin
std::string create_file_name(std::string category, std::optional<std::string> type, std::string number,
                             std::string extension)
{
    if(type!=nullopt)
        return category + "_" + *type + "_" + number + "." + extension;
    else
        return category + "_"  + number + "." + extension;
}

// Reaturn color in range [green; red] based on current and max values
// value in [0; value]
// 0       - green
// maxDiff - red
cv::Vec3b calcColor(float value, float maxValue)
{
    int color = (float)value / maxValue * 255;
    if(color > 255)
        color = 255;
    return cv::Vec3b(0, 255 - color, color);
}

pcl::visualization::PCLVisualizer::Ptr viewer;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered;
cv::Mat cells_map;
pcl::PointXYZRGB minP, maxP;

int cell_size_tr = 60;
int max_dispersion_tr = 7;
int max_z_diff_tr = 5;
float cell_size;
float max_dispersion;
float max_z_diff;

// Calculate statistic in cell
void calcCell(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,  Cell& cell)
{
    if(cell.indexes.empty())
        return;

    // Calculate meand and dispersion
    cell.z_mean /= cell.indexes.size();
    for (int index: cell.indexes)
    {
        float value = cloud->at(index).z - cell.z_mean;
        cell.z_dispersion += value * value;
    }
    cell.z_dispersion /= cell.indexes.size();
}


void colorCell(Cell& cell, GridCoord size, GridCoord coord, cv::Vec3b color)
{
    cells_map.at<cv::Vec3b>(size.row - 1 - coord.row, size.col - 1 - coord.col) = color;

    // Color points in cloud
    for (auto index: cell.indexes)
    {
        cloud_filtered->at(index).r = color[2];
        cloud_filtered->at(index).g = color[1];
        cloud_filtered->at(index).b = color[0];
    }
}

void drawCell(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB& min, const pcl::PointXYZRGB& max, int row, int col, float z, cv::Vec3b color)
{
    float x_min = min.x + row*cell_size;
    float x_max = min.x + (row+1)*cell_size;
    float y_min = min.y + col*cell_size;
    float y_max = min.y + (col+1)*cell_size;

    pcl::PointXYZRGB p;
    p.x = x_min;
    p.z = z;
    p.b = color[0];
    p.g = color[1];
    p.r = color[2];

    while(p.x<x_max)
    {
        p.y = y_min;
        while(p.y<y_max)
        {
            cloud->push_back(p);
            p.y+=0.05;
        }

        p.x+=0.05;
    }
}


void stepNext(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::vector<Cell>>& cells,  std::queue<GridCoord>& queue, GridCoord current, GridCoord next)
{
    GridCoord size(cells.size(), cells[0].size()); // TODO: fix it
    Cell& current_cell = cells[current.row][current.col];
    Cell& next_cell = cells[next.row][next.col];

    calcCell(cloud, next_cell);
    bool good = next_cell.z_dispersion < max_dispersion; //&& fabs(next_cell.z_mean - current_cell.z_mean) < max_z_diff;
    next_cell.visited = true;

    //if(next_cell.indexes.empty())
    //    return;

    cv::Vec3b color;
    if(good)
    {
        color = cv::Vec3b(0, 255, 0);
        queue.push(next);
    }
    else
    {
        color = cv::Vec3b(0, 0, 255);
    }

    if(!next_cell.indexes.empty())
        colorCell(next_cell, size, next, color);
    //drawCell(cloud, minP, maxP, next.row, next.col, next_cell.z_mean,  color);

}


void findFreeCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::vector<Cell>>& cells,  std::queue<GridCoord>& queue, GridCoord coord)
{
    int rows = cells.size();
    int cols = cells[0].size(); // TODO: fix it



    if(coord.row < rows-1 && !cells[coord.row+1][coord.col].visited)
        stepNext(cloud, cells, queue, coord, GridCoord(coord.row+1, coord.col));
    if(coord.col > 0 && !cells[coord.row][coord.col-1].visited)
        stepNext(cloud, cells, queue, coord, GridCoord(coord.row, coord.col-1));
    if(coord.col < cols-1 && !cells[coord.row][coord.col+1].visited)
        stepNext(cloud, cells, queue, coord, GridCoord(coord.row, coord.col+1));
}


void segmentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    double t0 = omp_get_wtime();

    // Get point cloud size
    pcl::getMinMax3D(*cloud, minP, maxP);
    float x_size = maxP.x - minP.x;
    float y_size = maxP.y - minP.y;

    // Get grid size and create array for grid
    int rows = (int)ceil(x_size / cell_size);
    int cols = (int)ceil(y_size / cell_size);
    std::vector<std::vector<Cell>> cells(rows, std::vector<Cell>(cols));

    // To draw grid image
    cells_map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));

    // Add points to the grid cells
    for(int i = 0; i<cloud->points.size(); i++)
    {
        int row = (int)((cloud->at(i).x - minP.x)/cell_size);
        int col = (int)((cloud->at(i).y - minP.y)/cell_size);
        cells[row][col].indexes.push_back(i);
        cells[row][col].z_mean += cloud->at(i).z;
    }

    int row0 = (int)((0 - minP.x)/cell_size);
    int col0 = (int)((0 - minP.y)/cell_size);
    GridCoord coord0(row0, col0);

    //while(row0 < cells.size() && cells[row0][col0].indexes.empty())
    //    row0++;

    std::queue<GridCoord> coords;
    calcCell(cloud, cells[coord0.row][coord0.col]);
    coords.push(coord0);

    while(!coords.empty())
    {
        findFreeCells(cloud, cells, coords,  coords.front());
        coords.pop();
    }

    if(viewer->contains("cloud"))
        viewer->removePointCloud("cloud");

    viewer->addPointCloud(cloud, "cloud");

    double duration = omp_get_wtime() - t0;
    cout << "elapsed: " << duration << endl;
}

void on_trackbar(int pos, void* userdata)
{
    // Properties from trackbars
    cell_size = cell_size_tr / 100.0f;
    max_dispersion = max_dispersion_tr / 10000.0f;
    max_z_diff = max_z_diff_tr / 100.0f;
    segmentCloud(cloud_filtered);
}

int main(int argc, char** argv)
{
    const string name = "umm_000005";

    const auto base_dir = kitti_dir / training_dir;
    const auto cloud_name = base_dir / velodyne_dir / (name+".bin");
    const auto calib_name = base_dir / calib_dir / (name+".txt");
    const auto rgb_right_name = base_dir / rgb_left_dir / (name+".png");

    // load data
    cloud = Kitti::load_cloud(cloud_name);
    if (cloud == nullptr)
        return -1;

    const auto calib = Kitti::load_calib(calib_name);
    if (calib == std::nullopt)
        return -1;

    //auto rgb_right = cv::imread(rgb_right_name.string());

    // Create viewer
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(10);
    filter.setStddevMulThresh(1.0);
    filter.filter(*cloud_filtered);

    cv::namedWindow("Control", cv::WINDOW_NORMAL);
    cv::resizeWindow("Control", 500, 500);
    cv::createTrackbar("Cell size", "Control", &cell_size_tr, 200, on_trackbar);
    cv::createTrackbar("Max dispersion", "Control", &max_dispersion_tr, 500, on_trackbar);
    cv::createTrackbar("Max Z diff", "Control", &max_z_diff_tr, 500, on_trackbar);

    on_trackbar(0, nullptr);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        cv::imshow("Control", cells_map);
        //cv::imshow("img", rgb_right);
        cv::waitKey(1);
    }

    return 0;
}