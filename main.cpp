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
cv::Mat cells_grid;
RoadSegmentation segmentator;

int cell_size_tr = 60;
int max_dispersion_tr = 7;
int max_z_diff_tr = 5;

// Draw cell in poiint cloud (for debug)
/*void drawCell(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB& min, const pcl::PointXYZRGB& max, int row, int col, float z, cv::Vec3b color)
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
}*/


void on_trackbar(int pos, void* userdata)
{
    // Properties from trackbars
    segmentator.set_params(cell_size_tr / 100.0f, max_dispersion_tr / 10000.0f, max_z_diff_tr / 100.0f);

    double t0 = omp_get_wtime();
    const TGridMap& grid = segmentator.calculate(cloud);
    double duration = omp_get_wtime() - t0;
    cout << "elapsed: " << duration << endl;

    cells_grid = cv::Mat(grid.rows(), grid.cols(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::flip(grid.image(), cells_grid, -1);

    if(viewer->contains("cloud"))
        viewer->removePointCloud("cloud");

    viewer->addPointCloud(grid.cloud(), "cloud");
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

    /*cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(10);
    filter.setStddevMulThresh(1.0);
    filter.filter(*cloud_filtered);*/

    cv::namedWindow("Control", cv::WINDOW_NORMAL);
    cv::resizeWindow("Control", 500, 500);
    cv::createTrackbar("Cell size", "Control", &cell_size_tr, 200, on_trackbar);
    cv::createTrackbar("Max dispersion", "Control", &max_dispersion_tr, 500, on_trackbar);
    cv::createTrackbar("Max Z diff", "Control", &max_z_diff_tr, 500, on_trackbar);

    on_trackbar(0, nullptr);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (30);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        cv::imshow("Control", cells_grid);
        cv::waitKey(1);
    }

    return 0;
}