# 文件等级：密一
# author:klug
# 献给我的心上人等待天使的妹妹
# start:230523
# last:230626

cmake_minimum_required(VERSION 3.14)

project(project_construct LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
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

add_library(project_construct SHARED
  project_construct_global.h
  ~/dieschwesterwaternderengle/ProjectRefactoring/project_construct.cpp
  ~/dieschwesterwaternderengle/ProjectRefactoring/project_construct.hpp
  ~/dieschwesterwaternderengle/ProjectRefactoring/project_cal.cpp
  ~/dieschwesterwaternderengle/ProjectRefactoring/project_cal.hpp
  ~/dieschwesterwaternderengle/ProjectRefactoring/project_img.cpp
  ~/dieschwesterwaternderengle/ProjectRefactoring/project_img.hpp
)

target_link_libraries(project_construct PRIVATE Qt${QT_VERSION_MAJOR}::Core
    ${OpenCV_LIBS}

    ${PCL_LIBRARIES}

    ${VTK_LIBRARIES}
    
    )

target_compile_definitions(project_construct PRIVATE project_construct_LIBRARY)
