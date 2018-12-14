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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = nullptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = nullptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud = nullptr;
pcl::PolygonMesh::Ptr mesh;
cv::Mat cells_grid;
RoadSegmentation segmentator;

int cell_size_tr = 60;
int max_dispersion_tr = 7;
int max_z_diff_tr = 5;
float cell_size, max_dispersion, max_z_diff;

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

void color_cell(const TGridMap& grid, GridCoord coord, const Color& color)
{
    const Cell& cell = grid.at(coord);

    if(!grid.at(coord).indexes.empty())
        cells_grid.at<Color>(grid.rows() - 1 - coord.row, grid.rows() - 1 - coord.col) = color;

    for(int index: cell.indexes)
    {
        display_cloud->at(index).x = grid.cloud_at(index).x;
        display_cloud->at(index).y = grid.cloud_at(index).y;
        display_cloud->at(index).z = grid.cloud_at(index).z;
        display_cloud->at(index).r = color[2];
        display_cloud->at(index).g = color[1];
        display_cloud->at(index).b = color[0];
    }
}

void display_obstacles(const TGridMap& grid)
{
    for(int row = 0; row<grid.rows(); row++)
    {
        for(int col = 0; col<grid.cols(); col++)
        {
            Color color;
            if(grid.at(row, col).obstacle == OBSTACLE)
                color = Color(0, 0, 255);
            else if(grid.at(row, col).obstacle == FREE)
                color = Color(0, 255, 0);
            else
                color = Color(255, 255, 255);

            color_cell(grid, GridCoord(row, col), color);
        }
    }
}

void display_z_mean(const TGridMap& grid)
{
    float color_per_z = 255.0f/(grid.max().z - grid.min().z);
    for(int row = 0; row<grid.rows(); row++)
    {
        for(int col = 0; col<grid.cols(); col++)
        {
            float c = (grid.at(row, col).z_mean - grid.min().z) * color_per_z;
            color_cell(grid, GridCoord(row, col), Color(0, 255 - c, c));
        }
    }
}

void display_z_mean_mesh(const TGridMap& grid)
{
    float color_per_z = 255.0f/(grid.max().z - grid.min().z);
    mesh = boost::make_shared<pcl::PolygonMesh>();
    display_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    int rows = grid.rows();
    int cols = grid.cols();

    for(int row = 0; row < rows; row++)
    {
        for(int col = 0; col < cols; col++)
        {
            float x = grid.min().x + row * grid.cell_size();
            float y = grid.min().y + col * grid.cell_size();
            float z = grid.at(row, col).z_mean;

            float c = (grid.at(row, col).z_mean - grid.min().z) * color_per_z;
            pcl::PointXYZRGB p(c, 255-c, 0);
            p.x = x;
            p.y = y;
            p.z = z;
            display_cloud->push_back(p);
        }
    }

    pcl::toPCLPointCloud2(*display_cloud, mesh->cloud);

    for(int row = 0; row < rows-1; row++)
    {
        for (int col = 0; col < cols-1; col++)
        {
            pcl::Vertices p1;
            p1.vertices.push_back(row*cols+col);
            p1.vertices.push_back(row*cols+col+1);
            p1.vertices.push_back((row+1)*cols+col);
            mesh->polygons.push_back(p1);
            pcl::Vertices p2;
            p2.vertices.push_back((row+1)*cols+col);
            p2.vertices.push_back((row+1)*cols+col+1);
            p2.vertices.push_back(row*cols+col+1);
            mesh->polygons.push_back(p2);
        }
    }

    /*pcl::PointXYZRGB p(255, 255, 255);
    p.x = -1; p.y = -1;
    display_cloud->push_back(p);
    p.x = -1; p.y =  1;
    display_cloud->push_back(p);
    p.x =  1; p.y =  1;
    display_cloud->push_back(p);
    p.x =  1; p.y = -1;
    display_cloud->push_back(p);

    pcl::toPCLPointCloud2(*display_cloud, mesh->cloud);

    {
        pcl::Vertices polygon;
        polygon.vertices.push_back(0);
        polygon.vertices.push_back(1);
        polygon.vertices.push_back(2);
        //polygon.vertices.push_back(3);
        mesh->polygons.push_back(polygon);
    }
    {
        pcl::Vertices polygon;
        polygon.vertices.push_back(0);
        polygon.vertices.push_back(3);
        polygon.vertices.push_back(2);
        //polygon.vertices.push_back(3);
        mesh->polygons.push_back(polygon);
    }*/
}

void display_z_dispersion(const TGridMap& grid)
{
    float color_per_z = 255.0f/max_dispersion;
    for(int row = 0; row<grid.rows(); row++)
    {
        for(int col = 0; col<grid.cols(); col++)
        {
            float c = grid.at(row, col).z_dispersion * color_per_z;
            if(c > 255) c = 255;
            color_cell(grid, GridCoord(row, col), Color(0, 255 - c, c));
        }
    }
}


void on_trackbar(int pos, void* userdata)
{
    // Properties from trackbars
    cell_size = cell_size_tr / 100.0f;
    max_dispersion = max_dispersion_tr / 10000.0f;
    max_z_diff = max_z_diff_tr / 100.0f;

    segmentator.set_params(cell_size, max_dispersion, max_z_diff);

    double t0 = omp_get_wtime();
    const TGridMap& grid = segmentator.calculate(cloud_filtered);
    double duration = omp_get_wtime() - t0;
    cout << "elapsed: " << duration << endl;

    display_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(grid.cloud_size(), 1, pcl::PointXYZRGB());
    cells_grid = cv::Mat(grid.rows(), grid.cols(), CV_8UC3, cv::Scalar(0, 0, 0));

    //display_obstacles(grid);
    //display_z_mean(grid);
    //display_z_dispersion(grid);
    display_z_mean_mesh(grid);

    if(display_cloud!=nullptr)
    {
        if (!viewer->contains("cloud"))
            viewer->addPointCloud(display_cloud, "cloud");
        else
            viewer->updatePointCloud(display_cloud, "cloud");
    }

    if(mesh!=nullptr)
    {
        if (!viewer->contains("mesh"))
            viewer->addPolygonMesh(*mesh, "mesh");
        else
            viewer->updatePolygonMesh(*mesh, "mesh");
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
        viewer->spinOnce (30);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        cv::imshow("Control", cells_grid);
        cv::waitKey(1);
    }

    return 0;
}