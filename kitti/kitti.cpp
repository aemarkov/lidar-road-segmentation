#include "kitti.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kitti::load_cloud(fs::path path)
{
    if(!fs::exists(path))
    {
        std::cout << "Error loading point cloud: " << path << "not found" << std::endl;
        return nullptr;
    }

    std::ifstream f(path, std::ios::in | std::ios::binary);

    // get the file size
    f.seekg(0, std::ios_base::end);
    size_t size = f.tellg();
    f.seekg(0, std::ios_base::beg);

    // load point cloud
    uint32_t points_count = size / (4 * sizeof(float)) + 1;
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(points_count, 1, pcl::PointXYZRGB());
    size_t i = 0;
    float intensity;

    pcl::PointXYZRGB point;
    point.r = 255;
    point.g = 255;
    point.b = 255;

    while(!f.eof())
    {
        f.read((char *) &point.x, 3*sizeof(float));
        f.read((char *) &intensity, sizeof(float));
        cloud->at(i) = point;
        i++;
    }

    return cloud;
}

std::optional<Kitti::Calib> Kitti::load_calib(fs::path path)
{
    if(!fs::exists(path))
    {
        std::cout << "Error loading calibration:" << path << "not found" << std::endl;
        return std::nullopt;
    }

    Kitti::Calib calib;
    std::ifstream f(path);
    read_matrix(f, calib.P0, 3, 4);
    read_matrix(f, calib.P1, 3, 4);
    read_matrix(f, calib.P2, 3, 4);
    read_matrix(f, calib.P3, 3, 4);
    read_matrix(f, calib.R0_rect, 3, 3);
    read_matrix(f, calib.Tr_velo_to_cam, 3, 4);
    read_matrix(f, calib.Tr_imu_to_velo, 3, 4);
    read_matrix(f, calib.Tr_cam_to_road, 3, 4);
    return calib;
}

// print calibration matrixes
void Kitti::print_calib(const Calib& calib)
{
    std::cout << "P0\n";
    print_mat(calib.P0);
    std::cout << "P1\n";
    print_mat(calib.P1);
    std::cout << "P2\n";
    print_mat(calib.P2);
    std::cout << "P3\n";
    print_mat(calib.P3);
    std::cout << "R0_rect\n";
    print_mat(calib.R0_rect);
    std::cout << "Tr_velo_to_cam\n";
    print_mat(calib.Tr_velo_to_cam);
    std::cout << "Tr_imu_to_velo\n";
    print_mat(calib.Tr_imu_to_velo);
    std::cout << "Tr_cam_to_road\n";
    print_mat(calib.Tr_cam_to_road);
}

// Read one matrix from KITTI calib file
void Kitti::read_matrix(std::ifstream& f, glm::mat4& mat, const int r, const int w)
{
    std::string wtf;
    f >> wtf;
    for(int row = 0; row < r; row++)
        for(int col = 0; col < w; col++)
            f >> mat[col][row];

    std::getline(f, wtf);

}

// Print matrix
void Kitti::print_mat(const glm::mat4& mat)
{
    for(int row = 0; row < 4; row++)
    {
        for(int col = 0; col < 4; col++)
            std::cout  << std::left <<  std::setw(10) << std::setprecision(2) << mat[col][row];
        std::cout << std::endl;
    }

    std::cout << std::endl;
}