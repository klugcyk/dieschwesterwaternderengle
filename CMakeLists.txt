# 文件等级:密一
# author:klug
# 献给我的心上人等待天使的妹妹，她是美丽女神阿尔忒弥斯的神女
# start:221129
# last:230627

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

# basler camera lib
include_directories(/opt/pylon/include)
include_directories(/opt/pylon/share/pylon/Samples/C++)
include_directories(/opt/pylon/share/pylon/Samples/C++/include)

# selbst
include_directories(~/buendia)
include_directories(~/dieschwesterwaternderengle)

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

        # windows
        ~/dieschwesterwaternderengle/ui_control/mainwindow.cpp
        ~/dieschwesterwaternderengle/ui_control/mainwindow.h
        ~/dieschwesterwaternderengle/ui_control/mainwindow.ui
        ~/dieschwesterwaternderengle/ui_control/img1.cpp
        ~/dieschwesterwaternderengle/ui_control/img1.h
        ~/dieschwesterwaternderengle/ui_control/img1.ui
        ~/dieschwesterwaternderengle/ui_control/zwei_construct_win.cpp
        ~/dieschwesterwaternderengle/ui_control/zwei_construct_win.h
        ~/dieschwesterwaternderengle/ui_control/zwei_construct_win.ui
        ~/dieschwesterwaternderengle/ui_control/cam_link.cpp
        ~/dieschwesterwaternderengle/ui_control/cam_link.h
        ~/dieschwesterwaternderengle/ui_control/cam_link.ui
        ~/dieschwesterwaternderengle/ui_control/cam_richt.cpp
        ~/dieschwesterwaternderengle/ui_control/cam_richt.h
        ~/dieschwesterwaternderengle/ui_control/cam_richt.ui
        #~/dieschwesterwaternderengle/ui_control/pc_show.cpp
        #~/dieschwesterwaternderengle/ui_control/pc_show.h
        #~/dieschwesterwaternderengle/ui_control/pc_show.ui

        # img algorithm
        ~/dieschwesterwaternderengle/StructureLight/construct_img.cpp
        ~/dieschwesterwaternderengle/StructureLight/construct_img.hpp
        ~/dieschwesterwaternderengle/StructureLight/construct_cal.cpp
        ~/dieschwesterwaternderengle/StructureLight/construct_cal.hpp
        ~/dieschwesterwaternderengle/StructureLight/construct.cpp
        ~/dieschwesterwaternderengle/StructureLight/construct.hpp
        ~/dieschwesterwaternderengle/StructureLight/zwei_construct_img.cpp
        ~/dieschwesterwaternderengle/StructureLight/zwei_construct_img.hpp
        ~/dieschwesterwaternderengle/StructureLight/zwei_construct_cal.cpp
        ~/dieschwesterwaternderengle/StructureLight/zwei_construct_cal.hpp
        ~/dieschwesterwaternderengle/StructureLight/zwei_construct.cpp
        ~/dieschwesterwaternderengle/StructureLight/zwei_construct.hpp
        ~/dieschwesterwaternderengle/CameraPose/cam_pose.cpp
        ~/dieschwesterwaternderengle/CameraPose/cam_pose.hpp
        ~/dieschwesterwaternderengle/CameraPose/pose_img.cpp
        ~/dieschwesterwaternderengle/CameraPose/pose_img.hpp
        ~/dieschwesterwaternderengle/ProjectRefactoring/project_img.hpp
        ~/dieschwesterwaternderengle/ProjectRefactoring/project_img.cpp
        ~/dieschwesterwaternderengle/ProjectRefactoring/project_cal.hpp
        ~/dieschwesterwaternderengle/ProjectRefactoring/project_cal.cpp
        ~/dieschwesterwaternderengle/ProjectRefactoring/project_construct.hpp
        ~/dieschwesterwaternderengle/ProjectRefactoring/project_construct.cpp
        ~/dieschwesterwaternderengle/ZweiAuge/zwei_auge_pre.cpp
        ~/dieschwesterwaternderengle/ZweiAuge/zwei_auge_pre.hpp
        ~/dieschwesterwaternderengle/ZweiAuge/zwei_auge_construct.cpp
        ~/dieschwesterwaternderengle/ZweiAuge/zwei_auge_construct.hpp

        # gitee
        #~/dieschwesterwaternderengle/StructureLight/constructimg.cpp
        #~/dieschwesterwaternderengle/StructureLight/constructimg.hpp
        #~/dieschwesterwaternderengle/StructureLight/constructcal.cpp
        #~/dieschwesterwaternderengle/StructureLight/constructcal.hpp
        #~/dieschwesterwaternderengle/StructureLight/construct_.cpp
        #~/dieschwesterwaternderengle/StructureLight/construct_.hpp


        # pointcloud algorithm
        ~/dieschwesterwaternderengle/pointcloud/construct_pointcloud.cpp
        ~/dieschwesterwaternderengle/pointcloud/construct_pointcloud.hpp

        # math
        ~/buendia/math/least_sqaure.cpp
        ~/buendia/math/least_sqaure.hpp
        ~/buendia/math/geometry.cpp
        ~/buendia/math/geometry.hpp

        # communication
        ~/dieschwesterwaternderengle/communication/communication.cpp
        ~/dieschwesterwaternderengle/communication/communication.hpp

        # outside control buendia
        ~/buendia/camera/camera.cpp
        ~/buendia/camera/camera.hpp
        ~/buendia/structurelight/structure_light.cpp
        ~/buendia/structurelight/structure_light.hpp
        ~/buendia/gene_operate/json_operate.cpp
        ~/buendia/gene_operate/json_operate.hpp
        ~/buendia/socket/socket.cpp
        ~/buendia/socket/socket.hpp
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

    /opt/pylon/lib/libgxapi.so
    /opt/pylon/lib/libPylonDataProcessing.so
    /opt/pylon/lib/libPylonDataProcessingGui.so
    /opt/pylon/lib/libXmlParser_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libpylon_TL_gtc.so
    /opt/pylon/lib/libpylonutility.so
    /opt/pylon/lib/libGenApi_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libpylonc.so
    /opt/pylon/lib/libLog_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libNodeMapData_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/liblog4cpp_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libPylonDataProcessingCore.so
    /opt/pylon/lib/libMathParser_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libFirmwareUpdate_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libuxapi.so
    /opt/pylon/lib/libpylon_TL_camemu.so
    /opt/pylon/lib/libGCBase_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libpylon_TL_gige.so
    /opt/pylon/lib/libpylonbase.so
    /opt/pylon/lib/pylon-libusb-1.0.so
    /opt/pylon/lib/libpylon_TL_usb.so
    /opt/pylon/lib/libpylonc.so.7.2
    /opt/pylon/lib/libuxapi.so.7.2
    /opt/pylon/lib/libgxapi.so.7.2
    /opt/pylon/lib/libpylonbase.so.7.2
    /opt/pylon/lib/libpylonutility.so.7.2
    /opt/pylon/lib/libgxapi.so.7.2.1
    /opt/pylon/lib/libpylonbase.so.7.2.1
    /opt/pylon/lib/libpylonc.so.7.2.1
    /opt/pylon/lib/libuxapi.so.7.2.1
    /opt/pylon/lib/libpylonutility.so.7.2.1

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
