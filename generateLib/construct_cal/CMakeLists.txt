# 文件等级：密一
# author:klug
# 献给杜尔西内娅德尔托博索
# start:230523
# last:230523

cmake_minimum_required(VERSION 3.14)

project(construct_cal LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

# pcl
find_package(PCL REQUIRED)
# opencv
find_package(OpenCV REQUIRED)
# eigen
find_package(Eigen3 REQUIRED)
# selbst
include_directories(~/buendia)
include_directories(~/dieschwesterwaternderengle)
include_directories(~/fernandadelcarpio)

# opencv
include_directories(/usr/local/include/opencv4)

# pcl
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# eigen
include_directories(${EIGEN3_INCLUDE_DIRS})

# VTK
find_package(VTK REQUIRED)
include_directories(/usr/include/vtk-7.1)

add_library(construct_cal SHARED
    construct_cal_global.h
    ~/dieschwesterwaternderengle/StructureLight/construct_cal.cpp
    ~/dieschwesterwaternderengle/StructureLight/construct_cal.hpp
    ~/dieschwesterwaternderengle/StructureLight/construct_img.cpp
    ~/dieschwesterwaternderengle/StructureLight/construct_img.hpp
)

target_link_libraries(construct_cal PRIVATE Qt${QT_VERSION_MAJOR}::Core)

target_compile_definitions(construct_cal PRIVATE CONSTRUCT_CAL_LIBRARY)
