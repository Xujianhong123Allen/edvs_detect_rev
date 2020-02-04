cmake_minimum_required(VERSION 3.5)
project(edvs_ros_simple)

find_package(catkin_simple REQUIRED)
catkin_simple()
# Default to C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(dvs_msgs REQUIRED)
find_package(libcaer REQUIRED)

INCLUDE_DIRECTORIES(/usr/include/libcaer)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

add_executable(edvs_ros_simple src/driver_node.cpp src/driver.cpp)

target_link_libraries(edvs_ros_simple
	caer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${libcaer_LIBRARIES}
)

install(TARGETS
  edvs_ros_simple
  DESTINATION lib/${PROJECT_NAME}
)
