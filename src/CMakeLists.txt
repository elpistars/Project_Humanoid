cmake_minimum_required(VERSION 3.7.2)
project(try)
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} )
add_executable(cam kamera.cpp)
target_link_libraries(cam dxl pthread ${OpenCV_LIBS})


