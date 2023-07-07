# 文件等级：密一
# author:klug
# 献给我的心上人等待天使的妹妹
# start:230608
# last:230608

cmake_minimum_required(VERSION 3.14)

project(communication LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
# find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

# opencv
find_package(OpenCV REQUIRED)

# selbst
include_directories(~/buendia)
include_directories(~/dieschwesterwaternderengle)

# opencv
include_directories(/usr/local/include/opencv4)

add_library(communication SHARED
  communication_global.h
  ~/dieschwesterwaternderengle/communication/communication.cpp
  ~/dieschwesterwaternderengle/communication/communication.hpp
  #~/buendia/socket/socket.cpp
  #~/buendia/socket/socket.hpp
)

target_link_libraries(communication PRIVATE #Qt${QT_VERSION_MAJOR}::Core
	~/sakura/lib/libsocket.so
)

target_compile_definitions(communication PRIVATE socket_LIBRARY)
