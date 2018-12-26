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
#include <functional>

#include "kitti/kitti.h"
#include "segmentation/RoadSegmentation.h"

using namespace std;
namespace fs = std::experimental::filesystem;

const fs::path kitti_dir = "/media/garrus/Seagate Expansion Drive/KITTI/Road/data_road";
const fs::path training_dir = "training";
const fs::path testing_dir = "testing";
const fs::path calib_dir = "calib";
const fs::path velodyne_dir = "velodyne";
const fs::path rgb_left_dir = "image_2";
const fs::path gt_dir = "gt_image_2";


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

pcl::visualization::PCLVisualizer::Ptr viewer = nullptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = nullptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr wtf_cloud = nullptr;
cv::Mat cells_grid;
RoadSegmentation segmentator;

int cell_size_tr = 50;
int max_dispersion_tr = 7;
int max_z_diff_tr = 5;
float cell_size, max_dispersion, max_z_diff;
int kernel_size = 10;
int iterations = 10;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointXYZRGB createVertex(float x, float y, float z, float r, float g, float b)
{
    pcl::PointXYZRGB p(r, g, b);
    p.x = x;
    p.y = y;
    p.z = z;

    return p;
}

pcl::PointXYZRGB createVertex(const pcl::PointXYZRGB& point, float r, float g, float b)
{
    pcl::PointXYZRGB p(r, g, b);
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;

    return p;
}

// Color cells in point cloud using specific function to calc cell color
pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud(const TGridMap& grid_map, const OccupancyGrid& occupancy_grid,
                                                     std::function<Color(const TGridMap&, const OccupancyGrid&, GridCoord coord)>get_color)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(grid_map.cloud_size(), 1, pcl::PointXYZRGB());
    for(int i = 0; i<grid_map.cloud_size(); i++)
    {
        GridCoord coord = grid_map.point_cell(i);
        Color color = get_color(grid_map, occupancy_grid, coord);
        cloud->at(i) = createVertex(grid_map.cloud_at(i), color[2], color[1], color[0]);
    }

    return cloud;
}

// Create mesh to display specific cell value (well, z_mean or z_max, nothing more can be displayed)
pcl::PolygonMesh::Ptr display_mesh(const TGridMap& grid_map, const OccupancyGrid& occupancy_grid,
                                   std::function<float(const TGridMap&, GridCoord)> get_z,
                                   std::function<Color(const TGridMap&, const OccupancyGrid&, GridCoord)> get_color)
{
    auto mesh = boost::make_shared<pcl::PolygonMesh>();
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    int rows = grid_map.rows();
    int cols = grid_map.cols();

    for(int row = 0; row < rows; row++)
    {
        for(int col = 0; col < cols; col++)
        {
            float z = get_z(grid_map, GridCoord(row, col));
            Color color = get_color(grid_map, occupancy_grid, GridCoord(row, col));

            cloud->push_back(createVertex(grid_map.min().x + row * grid_map.cell_size(),       grid_map.min().y + col * grid_map.cell_size(),       z, color[2], color[1], color[0]));
            cloud->push_back(createVertex(grid_map.min().x + row * grid_map.cell_size(),       grid_map.min().y + (col + 1) * grid_map.cell_size(), z, color[2], color[1], color[0]));
            cloud->push_back(createVertex(grid_map.min().x + (row + 1) * grid_map.cell_size(), grid_map.min().y + col * grid_map.cell_size(),       z, color[2], color[1], color[0]));
            cloud->push_back(createVertex(grid_map.min().x + (row + 1) * grid_map.cell_size(), grid_map.min().y + (col + 1) * grid_map.cell_size(), z, color[2], color[1], color[0]));
        }
    }

    pcl::toPCLPointCloud2(*cloud, mesh->cloud);

    for(int row = 0; row < rows-1; row++)
    {
        for (int col = 0; col < cols-1; col++)
        {
            int i = (row*cols +  col)*4;

            // Cell
            pcl::Vertices p1;
            p1.vertices.push_back(i);
            p1.vertices.push_back(i+1);
            p1.vertices.push_back(i+2);
            mesh->polygons.push_back(p1);
            pcl::Vertices p2;
            p2.vertices.push_back(i+3);
            p2.vertices.push_back(i+2);
            p2.vertices.push_back(i+1);
            mesh->polygons.push_back(p2);

            // vertial connection to the next col cell
            pcl::Vertices p3;
            p3.vertices.push_back(i+1);
            p3.vertices.push_back(i+4);
            p3.vertices.push_back(i+6);
            mesh->polygons.push_back(p3);
            pcl::Vertices p4;
            p4.vertices.push_back(i+1);
            p4.vertices.push_back(i+6);
            p4.vertices.push_back(i+3);
            mesh->polygons.push_back(p4);

            // vertial connection to the next row cell
            int j = ((row+1)*cols + col)*4;
            pcl::Vertices p5;
            p5.vertices.push_back(i+2);
            p5.vertices.push_back(i+3);
            p5.vertices.push_back(j);
            mesh->polygons.push_back(p5);
            pcl::Vertices p6;
            p6.vertices.push_back(j);
            p6.vertices.push_back(j+1);
            p6.vertices.push_back(i+3);
            mesh->polygons.push_back(p6);

        }
    }

    return mesh;
}

void display_image(const TGridMap& grid_map, const OccupancyGrid& occupancy_grid,
                   std::function<Color(const TGridMap&, const OccupancyGrid&, GridCoord)>get_color)
{
    for(size_t row = 0; row<grid_map.rows(); row++)
    {
        for(size_t col = 0; col<grid_map.cols(); col++)
        {
            Color color = get_color(grid_map, occupancy_grid, GridCoord(row, col));
            cells_grid.at<Color>(grid_map.rows() - 1 - row, grid_map.cols() - 1 - col) = color;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float z_mean(const TGridMap& grid, GridCoord coord) {  return grid.z_mean().at(coord); }
float z_max(const TGridMap& grid, GridCoord coord) {  return grid.z_max().at(coord); }

Color z_mean_color(const TGridMap& grid_map, const OccupancyGrid& occupancy_grid, GridCoord coord)
{
    float min = grid_map.min().z;
    float color_per_z = 255.0f/(max_z_diff - min);
    int color =  (grid_map.z_mean().at(coord) - min) * color_per_z;
    if(color > 255) color = 255;
    return Color(0, 255 - color, color);
}

Color z_dispersion_color(const TGridMap& grid_map, const OccupancyGrid& occupancy_grid, GridCoord coord)
{
    float color_per_z = 255.0f/max_dispersion;
    int color =  grid_map.z_dispersion().at(coord) * color_per_z;
    if(color > 255) color = 255;
    return Color(0, 255 - color, color);
}

Color obstacle_color(const TGridMap& grid_map, const OccupancyGrid& occupancy_grid, GridCoord coord)
{
    if(occupancy_grid.grid().at(coord) == OBSTACLE)
        return Color(0, 0, 255);
    else if(occupancy_grid.grid().at(coord) == FREE)
        return Color(0, 255, 0);

    return Color(255, 255, 255);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_trackbar(int pos, void* userdata)
{
    // Properties from trackbars
    cell_size = cell_size_tr / 100.0f;
    max_dispersion = max_dispersion_tr / 10000.0f;
    max_z_diff = max_z_diff_tr / 100.0f;

    if(kernel_size % 2 == 0)
        kernel_size++;

    segmentator.set_params(cell_size, max_dispersion, max_z_diff, GridCoord(kernel_size, kernel_size), iterations);

    double t0 = omp_get_wtime();
    auto [occupancyGrid, gridMap] = segmentator.calculate(cloud_filtered);
    double duration = omp_get_wtime() - t0;
    cout << "elapsed: " << duration << endl;

    cells_grid = cv::Mat(gridMap.rows(), gridMap.cols(), CV_8UC3, cv::Scalar(0, 0, 0));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PolygonMesh::Ptr mesh;

    //cloud = display_cloud(gridMap, occupancyGrid, z_mean_color);
    //display_image(gridMap, occupancyGrid, z_mean_color);

    //cloud = display_cloud(gridMap, occupancyGrid, z_dispersion_color);
    //display_image(gridMap, occupancyGrid, z_dispersion_color);

    cloud = display_cloud(gridMap, occupancyGrid, obstacle_color);
    display_image(gridMap, occupancyGrid, obstacle_color);

    //mesh = display_mesh(gridMap, occupancyGrid, z_mean, z_mean_color);
    //mesh = display_mesh(gridMap, occupancyGrid, z_max, z_mean_color);

    if(cloud!=nullptr)
    {
        if (viewer->contains("cloud"))
            viewer->removePointCloud("cloud");
        viewer->addPointCloud(cloud, "cloud");
    }

    /*if(cloud_filtered!=nullptr)
    {
        if (viewer->contains("cloud2"))
            viewer->removePointCloud("cloud2");
        viewer->addPointCloud(cloud_filtered, "cloud2");
    }*/

    if(mesh!=nullptr)
    {
        if (viewer->contains("mesh"))
            viewer->removePolygonMesh("mesh");
        viewer->addPolygonMesh(*mesh, "mesh");
    }
}

int main(int argc, char** argv)
{
    const string name = "um_000015";

    const auto base_dir = kitti_dir / training_dir;
    const auto cloud_name = base_dir / velodyne_dir / (name+".bin");
    const auto calib_name = base_dir / calib_dir / (name+".txt");
    const auto rgb_right_name = base_dir / rgb_left_dir / (name+".png");


    // load data
    auto cloud = Kitti::load_cloud(cloud_name);
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
    filter.setStddevMulThresh(1);
    filter.filter(*cloud_filtered);

    cv::namedWindow("Control", cv::WINDOW_NORMAL);
    cv::resizeWindow("Control", 500, 500);
    cv::createTrackbar("Cell size", "Control", &cell_size_tr, 200, on_trackbar);
    cv::createTrackbar("Max dispersion", "Control", &max_dispersion_tr, 500, on_trackbar);
    cv::createTrackbar("Max Z diff", "Control", &max_z_diff_tr, 500, on_trackbar);
    cv::createTrackbar("Kernel size", "Control", &kernel_size, 15, on_trackbar);
    cv::createTrackbar("Iterations", "Control", &iterations, 10, on_trackbar);

    on_trackbar(0, nullptr);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (30);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        cv::imshow("Control", cells_grid);
        cv::waitKey(1);
    }

    return 0;
}