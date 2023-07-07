# 文件等级：密一
# author:klug
# 献给不喜欢我的弗里德里希冯海因洛特
# start:230523
# last:230606

cmake_minimum_required(VERSION 3.14)

project(zweiauge LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
#find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

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

add_library(zweiauge SHARED
  zweiauge_global.h
  ~/dieschwesterwaternderengle/ZweiAuge/zwei_auge_construct.cpp
  ~/dieschwesterwaternderengle/ZweiAuge/zwei_auge_construct.hpp
  ~/dieschwesterwaternderengle/ZweiAuge/zwei_auge_pre.cpp
  ~/dieschwesterwaternderengle/ZweiAuge/zwei_auge_pre.hpp
)

target_link_libraries(zweiauge PRIVATE #Qt${QT_VERSION_MAJOR}::Core
    ${OpenCV_LIBS}

    ${PCL_LIBRARIES}

    ${VTK_LIBRARIES}
    )

target_compile_definitions(zweiauge PRIVATE zweiauge_LIBRARY)
