cmake_minimum_required(VERSION 3.12)
project(road_segmentation)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "-fopenmp")

find_package(PCL 1.9 REQUIRED COMPONENTS
        common
        io
        visualization
        filters)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} .)
add_definitions(${PCL_DEFINITIONS})


add_executable(road_segmentation main.cpp kitti/kitti.cpp kitti/kitti.h segmentation/RoadSegmentation.cpp segmentation/RoadSegmentation.h segmentation/GridMap.h OccupancyGrid/OccupancyGrid.cpp OccupancyGrid/OccupancyGrid.h GridCoord.h)
target_link_libraries(road_segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} stdc++fs)