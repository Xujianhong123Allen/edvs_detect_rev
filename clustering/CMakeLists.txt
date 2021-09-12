cmake_minimum_required(VERSION 3.5)
project(clustering)

find_package(catkin_simple REQUIRED)
catkin_simple()
# Default to C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(dvs_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(clustering src/clustering_node.cpp src/clustering.cpp)

target_link_libraries(clustering
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Opencv_LIBRARIES}
)

install(TARGETS
  clustering
  DESTINATION lib/${PROJECT_NAME}
)
