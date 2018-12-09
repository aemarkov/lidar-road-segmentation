//
// KITTI helpers
//

#ifndef ROAD_SEGMENTATION_KITTI_H
#define ROAD_SEGMENTATION_KITTI_H

#include <experimental/filesystem>
#include <glm/glm.hpp>
#include <pcl/common/common_headers.h>
#include <boost/make_shared.hpp>
#include <fstream>
#include <iomanip>
#include <optional>

namespace fs = std::experimental::filesystem;

class Kitti
{
public:
    struct Calib
    {
        glm::mat4 P0;                 // Projection matrix for left grayscale camera in rectified coordinates
        glm::mat4 P1;                 // Projection matrix for right grayscale camera in rectified coordinates
        glm::mat4 P2;                 // Projection matrix for left color camera in rectified coordinates
        glm::mat4 P3;                 // Projection matrix for right color camera in rectified coordinates
        glm::mat4 R0_rect;            // Rotation from non-rectified to rectified camera coordinate system
        glm::mat4 Tr_velo_to_cam;     // Rigid transformation from Velodyne to (non-rectified) camera coordinates
        glm::mat4 Tr_imu_to_velo;     // Rigid transformation from IMU to Velodyne coordinates
        glm::mat4 Tr_cam_to_road;     // Rigid transformation from (non-rectified) camera to road coordinates
    };

    /**
     * Read point cloud from KITT .bin file
     * @param path path to the .bin file
     * @return pointer to the PointCloud
     */
    // TODO: Now I make pcl::PointXYZRGB for debug purpose
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_cloud(fs::path path);

    /**
     * Read calibration matrices
     * @param path path to calib.txt file
     * @return struct with all matrices
     */
    static std::optional<Calib> load_calib(fs::path path);

    /**
     * Print calibration matrices for debug purposes
     * @param calib
     */
    static void print_calib(const Calib& calib);

private:
    // Read one matrix from KITTI calib file
    static void read_matrix(std::ifstream& f, glm::mat4& mat, const int r, const int w);

    // Print matrix
    static void print_mat(const glm::mat4& mat);
};

#endif //ROAD_SEGMENTATION_KITTI_H
