#include <iostream>
#include <string>
#include <experimental/filesystem>
#include <optional>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

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
cv::Mat cells_map;

int cell_size_tr = 60;
int max_value_tr = 20;

void cloudWtf()
{
    double t0 = omp_get_wtime();

    // Properties from trackbars
    const float cell_size = cell_size_tr / 100.0f;
    const float max_value = max_value_tr / 10000.0f;

    // Get point cloud size
    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*cloud, min, max);
    float x_size = max.x - min.x;
    float y_size = max.y - min.y;

    // Get grid size and create array for grid
    int rows = (int)ceil(x_size / cell_size);
    int cols = (int)ceil(y_size / cell_size);
    std::vector<Cell> cells(rows*cols);

    // To draw grid image
    cells_map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));

    // Add points to the grid cells
    for(int i = 0; i<cloud->points.size(); i++)
    {
        int row = (int)((cloud->at(i).x - min.x)/cell_size);
        int col = (int)((cloud->at(i).y - min.y)/cell_size);
        cells[row*cols + col].indexes.push_back(i);
        cells[row*cols + col].z_mean += cloud->at(i).z;
    }

    // Calculate and show dispersion
    for(int i = 0; i<cells.size(); i++)
    {
        int row = i/cols;
        int col = i%cols;

        if(!cells[i].indexes.empty())
        {
            cells[i].z_mean /= cells[i].indexes.size();
            for (auto index: cells[i].indexes)
            {
                float value = cloud->at(index).z - cells[i].z_mean;
                cells[i].z_dispersion += value * value;
            }

            cells[i].z_dispersion /= cells[i].indexes.size();

            // Draw cells on picture
            cv::Vec3b color = calcColor(cells[i].z_dispersion, max_value);
            cells_map.at<cv::Vec3b>(rows-1-row, cols-1-col) = color;

            // Color points in cloud
            for(auto index: cells[i].indexes)
            {
                cloud->at(index).r = color[2];
                cloud->at(index).g = color[1];
                cloud->at(index).b = color[0];
            }
        }
    }

    if(viewer->contains("cloud"))
        viewer->removePointCloud("cloud");

    viewer->addPointCloud(cloud, "cloud");

    double duration = omp_get_wtime() - t0;
    cout << "elapsed: " << duration << endl;
}

void on_trackbar(int pos, void* userdata)
{
    cloudWtf();
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
    //viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");

    cv::namedWindow("Control", cv::WINDOW_NORMAL);
    cv::resizeWindow("Control", 500, 500);
    cv::createTrackbar("Cell size", "Control", &cell_size_tr, 200, on_trackbar);
    cv::createTrackbar("Max value", "Control", &max_value_tr, 500, on_trackbar);

    cloudWtf();

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