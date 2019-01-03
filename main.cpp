#include <iostream>
#include <string>
#include <experimental/filesystem>
#include <optional>
#include <queue>
#include <functional>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <boost/make_shared.hpp>
#include <opencv2/opencv.hpp>
#include <omp.h>

#include "kitti/kitti.h"
#include "segmentation/RoadSegmentation.h"

using namespace std;
namespace fs = std::experimental::filesystem;
using Color = cv::Vec3b;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointXYZRGB createVertex(float x, float y, float z, float r, float g, float b)
{
   pcl::PointXYZRGB p(r, g, b);
    p.x = x;
    p.y = y;
    p.z = z;

    return p;
}

pcl::PointXYZRGB createVertex(const pcl::PointXYZ& point, float r, float g, float b)
{
    pcl::PointXYZRGB p(r, g, b);
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;

    return p;
}

// Color cells in point cloud using specific function to calc cell color
pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud(const TGridMap& grid,  std::function<Color(const TGridMap&, GridCoord coord)>get_color)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(grid.cloud_size(), 1, pcl::PointXYZRGB());
    for(int i = 0; i<grid.cloud_size(); i++)
    {
        GridCoord coord = grid.point_cell(i);
        Color color = get_color(grid, coord);
        cloud->at(i) = createVertex(grid.cloud_at(i), color[2], color[1], color[0]);
    }

    return cloud;
}


cv::Mat display_image(const TGridMap& grid, std::function<Color(const TGridMap&, GridCoord)>get_color)
{
    auto cells_grid = cv::Mat(grid.rows(), grid.cols(), CV_8UC3, cv::Scalar(0, 0, 0));
    for(size_t row = 0; row<grid.rows(); row++)
    {
        for(size_t col = 0; col<grid.cols(); col++)
        {
            Color color = get_color(grid, GridCoord(row, col));
            cells_grid.at<Color>(grid.rows() - 1 - row, grid.cols() - 1 - col) = color;
        }
    }

    return cells_grid;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Color obstacle_color(const TGridMap& grid, GridCoord coord)
{
    if(grid.obstacles().at(coord) == OBSTACLE)
        return Color(0, 0, 255);
    else if(grid.obstacles().at(coord) == FREE)
        return Color(0, 255, 0);

    return Color(255, 255, 255);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/garrus/bags/street.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    auto viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    RoadSegmentation segmentator(0.25, 0.05);
    auto grid = segmentator.calculate(cloud);

    // view cloud
    auto d_cloud = display_cloud(grid, obstacle_color);
    viewer->addPointCloud(d_cloud, "display_cloud");

    // view image
    auto image = display_image(grid, obstacle_color);
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    cv::resizeWindow("Image", 500, 500);
    cv::imshow("Image", image);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (30);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        cv::waitKey(1);
    }

    return 0;
}

/*

const fs::path kitti_dir = "/media/garrus/Seagate Expansion Drive/KITTI/Road/data_road";
const fs::path training_dir = "training";
const fs::path testing_dir = "testing";
const fs::path calib_dir = "calib";
const fs::path velodyne_dir = "velodyne";
const fs::path rgb_left_dir = "image_2";
const fs::path gt_dir = "gt_image_2";


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

cv::namedWindow("Control", cv::WINDOW_NORMAL);
cv::resizeWindow("Control", 500, 500);*/