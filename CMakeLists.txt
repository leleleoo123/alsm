cmake_minimum_required(VERSION 2.8.9)
project(aslam)

find_package( OpenCV REQUIRED )
find_package(Pangolin 0.4)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

#add source files
FILE(GLOB SRCS *cpp *hpp *h)

add_executable(aslam ${SRCS} )
target_link_libraries(aslam ${OpenCV_LIBS} )
target_link_libraries(aslam ${Pangolin_LIBRARIES})
target_link_libraries(aslam ${CERES_LIBRARIES})