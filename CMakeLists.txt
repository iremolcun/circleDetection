cmake_minimum_required(VERSION 3.10)
project(EdgeHoughDetection)

find_package(OpenCV REQUIRED)

add_executable(edge_hough main.cpp)
target_link_libraries(edge_hough ${OpenCV_LIBS})
