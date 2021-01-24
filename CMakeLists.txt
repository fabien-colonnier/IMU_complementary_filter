cmake_minimum_required(VERSION 2.8.3)
project(complementary_filter)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -O0 -g -fsanitize=address")

find_package(Eigen3 REQUIRED)
include_directories(INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
find_package(Boost REQUIRED COMPONENTS system thread)

# make the executable
cs_add_executable(${PROJECT_NAME}_node
  src/main_node.cpp
  src/complementary_filter_ros.cpp
  src/complementary_filter.cpp
  src/utilities.cpp
)


# link the executable to the necesarry libs
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Install the nodelet library
cs_install()


