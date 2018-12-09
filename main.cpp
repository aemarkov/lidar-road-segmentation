#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <experimental/filesystem>
#include <boost/make_shared.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
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

struct RGB
{
    int r, g, b;
    RGB(){}
    RGB(int r, int g, int b)
    {
        this->r = r;
        this->g = g;
        this->b = b;
    }
};

struct Cell
{
    std::vector<int> indexes;
    float z_mean = 0;
    float z_dispersion = 0;
    int col, row;
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
// diff in [0; maxDiff]
// 0       - green
// maxDiff - red
RGB calcColor(float value, float maxValue)
{
    int color = (float)value / maxValue * 255;
    if(color > 255)
        color = 255;
    return RGB(color, 255 - color, 0);
}

pcl::visualization::PCLVisualizer::Ptr viewer;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

int cell_size_tr = 100;
int max_value_tr = 20;
const float cell_draw_step = 0.1;

void drawCell(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB& min, const pcl::PointXYZRGB& max, float cell_size, int row, int col, const RGB& color)
{
    float x_min = min.x + row*cell_size;
    float x_max = min.x + (row+1)*cell_size;
    float y_min = min.y + col*cell_size;
    float y_max = min.y + (col+1)*cell_size;

    pcl::PointXYZRGB p;
    p.x = x_min;
    p.r = color.r;
    p.g = color.g;
    p.b = color.b;

    while(p.x<x_max)
    {
        p.y = y_min;
        while(p.y<y_max)
        {
            cloud->push_back(p);
            p.y+=cell_draw_step;
        }

        p.x+=cell_draw_step;
    }
}

void cloudWtf()
{
    // Find bounding box
    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*cloud, min, max);
    //viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, 1.0, 0.0, 0.0, "box", 0);
    //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "box");

    const float cell_size = cell_size_tr / 100.0f;
    const float max_value = max_value_tr / 10000.0f;

    double t0 = omp_get_wtime();

    float x_size = max.x - min.x;
    float y_size = max.y - min.y;

    int rows = ceil(x_size / cell_size);
    int cols = ceil(y_size / cell_size);
    std::vector<Cell> cells(rows*cols);

    for(int i = 0; i<cloud->points.size(); i++)
    {
        int row = (cloud->at(i).x - min.x)/cell_size;
        int col = (cloud->at(i).y - min.y)/cell_size;
        cells[row*cols + col].indexes.push_back(i);
        cells[row*cols + col].z_mean += cloud->at(i).z;
    }

    auto cloud_cells = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    for(int i = 0; i<cells.size(); i++)
    {
        int row = i/cols;
        int col = i%cols;

        if(cells[i].indexes.size()>0)
        {
            cells[i].z_mean /= cells[i].indexes.size();
            for (auto index: cells[i].indexes)
            {
                float value = cloud->at(index).z - cells[i].z_mean;
                cells[i].z_dispersion += value * value;
            }

            cells[i].z_dispersion /= cells[i].indexes.size();

            RGB color = calcColor(cells[i].z_dispersion, max_value);
            drawCell(cloud_cells, min, max, cell_size, row, col, color);
        }
        else
        {
            RGB color;
            color.r = 50;
            color.g = 50;
            color.b = 50;
            drawCell(cloud_cells, min, max, cell_size, row, col, color);
        }
    }

    if(viewer->contains("cloud"))
        viewer->removePointCloud("cloud");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");

    if(viewer->contains("cloud_cells"))
        viewer->removePointCloud("cloud_cells");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cells, "cloud_cells");

    double duration = omp_get_wtime() - t0;
    cout << "elapsed: " << duration << endl;
}

void on_trackbar(int pos, void* userdata)
{
    cloudWtf();
}

int main(int argc, char** argv)
{
    const string cat = "um";
    const string type = "road";
    const string number = "000054";

    const auto base_dir = kitti_dir / training_dir;
    const auto calib_name = base_dir / calib_dir / create_file_name(cat, nullopt, number, "txt");
    const auto cloud_name = base_dir / velodyne_dir / create_file_name(cat, nullopt, number, "bin");
    //const auto gt_name = base_dir / gt_dir / create_file_name(cat, type, number, "png");
    const auto rgb_right_name = base_dir / rgb_left_dir / create_file_name(cat, nullopt, number, "png");

    // load data
    cloud = Kitti::load_cloud(cloud_name);
    if(cloud == nullptr)
        return -1;

    const auto calib = Kitti::load_calib(calib_name);
    if(calib == std::nullopt)
        return -1;

    auto rgb_right = cv::imread(rgb_right_name.string());
    //auto gt = cv::imread(gt_name.string());

    // Create viewer
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    cv::namedWindow("Control");
    cv::createTrackbar("Cell size", "Control", &cell_size_tr, 200, on_trackbar);
    cv::createTrackbar("Max value", "Control", &max_value_tr, 500, on_trackbar);

    cloudWtf();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        //cv::imshow("Control", rgb_right);
        cv::waitKey(1);
    }

    return 0;
}