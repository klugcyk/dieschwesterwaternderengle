# 文件等级:密一
# author:klug
# 献给我的心上人等待天使的妹妹，她是美丽女神阿尔忒弥斯的神女
# start:221129
# last:230731

cmake_minimum_required(VERSION 3.5)

project(dieschwesterwaternderengle VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

# pcl
find_package(PCL REQUIRED)
# opencv
find_package(OpenCV REQUIRED)
# eigen
find_package(Eigen3 REQUIRED)

# selbst
include_directories(~/undb)
include_directories(~/dieschwesterwaternderengle)
include_directories(/usr/include)

# opencv
include_directories(${OpenCV_INCLUDE_DIRS})

# pcl
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# eigen
include_directories(${EIGEN3_INCLUDE_DIRS})

# VTK
find_package(VTK REQUIRED)
include_directories(/usr/include/vtk-7.1)

set(PROJECT_SOURCES
    main.cpp
    source.hpp

    # img algorithm
    ~/dieschwesterwaternderengle/StructureLight/construct_img.cpp
    ~/dieschwesterwaternderengle/StructureLight/construct_img.hpp
    ~/dieschwesterwaternderengle/StructureLight/construct_cal.cpp
    ~/dieschwesterwaternderengle/StructureLight/construct_cal.hpp
    ~/dieschwesterwaternderengle/StructureLight/construct.cpp
    ~/dieschwesterwaternderengle/StructureLight/construct.hpp

    #~/dieschwesterwaternderengle/StructureLight/zwei_construct_img.cpp
    #~/dieschwesterwaternderengle/StructureLight/zwei_construct_img.hpp
    #~/dieschwesterwaternderengle/StructureLight/zwei_construct_cal.cpp
    #~/dieschwesterwaternderengle/StructureLight/zwei_construct_cal.hpp
    #~/dieschwesterwaternderengle/StructureLight/zwei_construct.cpp
    #~/dieschwesterwaternderengle/StructureLight/zwei_construct.hpp

    ~/undb/math/least_sqaure.hpp
    ~/undb/math/least_sqaure.cpp
    ~/undb/math/geometry.cpp
    ~/undb/math/geometry.hpp
    ~/undb/data_array/data_array.hpp
    ~/undb/data_array/data_array.cpp
    ~/undb/cameraGene/cameraGene.hpp
    ~/undb/cameraGene/cameraGene.cpp
    ~/undb/meinCV/generateKernel.hpp
    ~/undb/meinCV/generateKernel.cpp

    ~/undb/json_operate/json_operate.cpp
    ~/undb/json_operate/json_operate.hpp

)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(dieschwesterwaternderengle
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET dieschwesterwaternderengle APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(dieschwesterwaternderengle SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(dieschwesterwaternderengle
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(dieschwesterwaternderengle PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    pthread

    ${OpenCV_LIBS}

    ${PCL_LIBRARIES}

    ${VTK_LIBRARIES}
)

set_target_properties(dieschwesterwaternderengle PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(dieschwesterwaternderengle)
endif()
