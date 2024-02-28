cmake_minimum_required(VERSION 2.8)
project( ObjectDetect )
find_package( OpenCV REQUIRED )
add_executable( ObjectDetect ObjectDetect.cpp my_blobs.cpp)
target_link_libraries( ObjectDetect ${OpenCV_LIBS} )

