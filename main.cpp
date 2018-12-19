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

// color cell in point cloud (and image)
void color_cell(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const TGridMap& grid, GridCoord coord, const Color& color)
{

    cells_grid.at<Color>(grid.rows() - 1 - coord.row, grid.cols() - 1 - coord.col) = color;

    for(int index: grid.indexes(coord))
    {
        cloud->at(index).x = grid.cloud_at(index).x;
        cloud->at(index).y = grid.cloud_at(index).y;
        cloud->at(index).z = grid.cloud_at(index).z;
        cloud->at(index).r = color[2];
        cloud->at(index).g = color[1];
        cloud->at(index).b = color[0];
    }
}

// Show obstacles with color
pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_obstacles(const OccupancyGrid& occupancyGrid, const TGridMap& gridMap)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(gridMap.cloud_size(), 1, pcl::PointXYZRGB());

    for(int row = 0; row<occupancyGrid.rows(); row++)
    {
        for(int col = 0; col<occupancyGrid.cols(); col++)
        {
            Color color;
            if(occupancyGrid.at(row, col) == OBSTACLE)
                color = Color(0, 0, 255);
            else if(occupancyGrid.at(row, col) == FREE)
                color = Color(0, 255, 0);
            else
                color = Color(255, 255, 255);

            color_cell(cloud, gridMap, GridCoord(row, col), color);
        }
    }

    return cloud;
}

// Show z-mean with color
pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_z_mean(const TGridMap& grid)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(grid.cloud_size(), 1, pcl::PointXYZRGB());
    float color_per_z = 255.0f/(max_z_diff - grid.min().z);

    for(int row = 0; row<grid.rows(); row++)
    {
        for(int col = 0; col<grid.cols(); col++)
        {
            float c = (grid.z_mean(row, col) - grid.min().z) * color_per_z;
            color_cell(cloud, grid, GridCoord(row, col), Color(0, 255 - c, c));
        }
    }

    return cloud;
}

// Show z-dispersion with color
pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_z_dispersion(const TGridMap& grid)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(grid.cloud_size(), 1, pcl::PointXYZRGB());
    float color_per_z = 255.0f/max_dispersion;

    for(int row = 0; row<grid.rows(); row++)
    {
        for(int col = 0; col<grid.cols(); col++)
        {
            float wtf = grid.z_dispersion(row, col);
            float c = wtf * color_per_z;

            if(c > 255) c = 255;

            color_cell(cloud, grid, GridCoord(row, col), Color(0, 255 - c, c));
        }
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_wtf(const TGridMap &gridMap)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(gridMap.cloud_size(), 1, pcl::PointXYZRGB());
    int cnt = 0;

    for(int row = 0; row<gridMap.rows(); row++)
    {
        for (int col = 0; col < gridMap.cols(); col++)
        {
            int c = ((row+col) % 2 == 0) ? 255 : 0;
            const auto& indexes = gridMap.indexes(row, col);
            for(int index: indexes)
            {
                auto point = gridMap.cloud_at(index);
                point.r = c;
                point.g = 255-c;
                point.b = 0;

                cloud->at(cnt) = point;
                cnt++;
            }
        }
    }

    cout << gridMap.cloud_size();
    cout << cnt << endl;

    return cloud;
}

pcl::PointXYZRGB createVertex(float x, float y, float z, float r, float g, float b)
{
    pcl::PointXYZRGB p(r, g, b);
    p.x = x;
    p.y = y;
    p.z = z;

    return p;
}

// Show z-mean with mesh
pcl::PolygonMesh::Ptr display_z_mean_mesh(const TGridMap& grid)
{
    float color_per_z = 255.0f/(grid.max().z - grid.min().z);
    auto mesh = boost::make_shared<pcl::PolygonMesh>();
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    int rows = grid.rows();
    int cols = grid.cols();

    for(int row = 0; row < rows; row++)
    {
        for(int col = 0; col < cols; col++)
        {
            float z = grid.z_max(row, col);
            float c = (grid.z_max(row, col) - grid.min().z) * color_per_z;


            cloud->push_back(createVertex(grid.min().x + row * grid.cell_size(),       grid.min().y + col * grid.cell_size(),       z, c, 255 - c, 0));
            cloud->push_back(createVertex(grid.min().x + row * grid.cell_size(),       grid.min().y + (col + 1) * grid.cell_size(), z, c, 255 - c, 0));
            cloud->push_back(createVertex(grid.min().x + (row + 1) * grid.cell_size(), grid.min().y + col * grid.cell_size(),       z, c, 255 - c, 0));
            cloud->push_back(createVertex(grid.min().x + (row + 1) * grid.cell_size(), grid.min().y + (col + 1) * grid.cell_size(), z, c, 255 - c, 0));
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud;
    pcl::PolygonMesh::Ptr mesh;

    //display_cloud = display_obstacles(occupancyGrid, gridMap);
    //display_cloud = display_z_mean(gridMap);
    display_cloud = display_z_dispersion(gridMap);
    //display_cloud = display_wtf(gridMap);
    //mesh = display_z_mean_mesh(gridMap);

    if(display_cloud!=nullptr)
    {
        if (viewer->contains("cloud"))
            viewer->removePointCloud("cloud");
        viewer->addPointCloud(display_cloud, "cloud");
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
    const string name = "umm_000005";

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
    filter.setStddevMulThresh(1.0);
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