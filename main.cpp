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
RGB calcColor(float diff, float maxDiff)
{
    int color = (float)diff / maxDiff * 255;
    if(color > 255)
        color = 255;
    return RGB(color, 255 - color, 0);
}

pcl::visualization::PCLVisualizer::Ptr viewer;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

// for neighbour search
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;
int trRadius = 53;
int trMaxDiff = 15;

// Process point cloud and update in visualizer
void wtfCloud()
{
    double start = omp_get_wtime();

    float radius = (float)trRadius / 100.0f;
    float maxDiff = (float)trMaxDiff / 10000.0f;

#pragma omp parallel for
    for(int i = 0; i<cloud->points.size(); i++)
    {
        std::vector<int> neighboursIdx;
        std::vector<float> neighboursDists;
        if(tree->radiusSearch(i, radius, neighboursIdx, neighboursDists)>1)
        {
            float m = 0; // average
            float d = 0; // dispersion
            for (int j = 0; j < neighboursIdx.size(); j++)
                m += cloud->at(neighboursIdx[j]).z;

            m /= neighboursIdx.size();

            for (int j = 0; j < neighboursIdx.size(); j++)
            {
                float s = cloud->at(neighboursIdx[j]).z - m;
                d += s*s;
            }

            d /= neighboursIdx.size();
            RGB color = calcColor(d, maxDiff);
            cloud->at(i).r = color.r;
            cloud->at(i).g = color.g;
            cloud->at(i).b = color.b;
        }
    }

    double end = omp_get_wtime();
    cout << "Elapsed: " << end - start << endl;

    if(viewer->contains("cloud"))
        viewer->removePointCloud("cloud");

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
}

void on_trackbar(int pos, void* userdata)
{
    wtfCloud();
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

    // Initialize neighbour searching stuff
    tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    tree->setInputCloud(cloud);

    // Create viewer
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    // create trackbars
    cv::namedWindow("Control");
    cv::createTrackbar("Radius", "Control", &trRadius, 100, on_trackbar);
    cv::createTrackbar("Max diff", "Control", &trMaxDiff, 100, on_trackbar);

    /*auto pc2 = boost::make_shared<pcl::PCLPointCloud2>();
    auto pc2_filtered = boost::make_shared<pcl::PCLPointCloud2>();
    pcl::toPCLPointCloud2(*cloudRaw, *pc2);

    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(pc2);
    filter.setLeafSize(0.1, 0.1, 0.1);
    filter.filter(*pc2_filtered);

    cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromPCLPointCloud2(*pc2_filtered, *cloud);*/

    //wtfCloud();
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        cv::imshow("Control", rgb_right);
        cv::waitKey(1);
    }

    return 0;
}

/*auto cloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    auto mat = glm::inverse(calib.P2 * calib.R0_rect * glm::inverse(calib.Tr_cam_to_road));
    for(int row = 0; row < gt.rows; row++)
    {
        for(int col = 0; col < gt.cols; col++)
        {
            glm::vec4 p_img(row, col, 0, 1);
            glm::vec4 p_road = mat * p_img;
            pcl::PointXYZRGB p(255, 0, 0);
            p.x = p_road.x;
            p.y = p_road.y;
            p.z = p_road.z;
            cloud2->push_back(p);
        }
    }*/
