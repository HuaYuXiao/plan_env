cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
        tf
        octomap_ros
        octomap_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs tf
 DEPENDS OpenCV
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/ThirdParty/sdf_tools/include
        ${PROJECT_SOURCE_DIR}/ThirdParty/arc_utilities/include
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env
        src/sdf_map.cpp
        src/edt_environment.cpp
        src/global_point_sdf.cpp
    src/grid_map.cpp 
    src/raycast.cpp
    src/obj_predictor.cpp 
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
        ${OCTOMAP_LIBRARIES}
        ${Boost_LIBRARIES}
    )  

add_executable(read_pub_ot
        src/read_pub_ot.cpp)
target_link_libraries(read_pub_ot
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
)

add_executable(obj_generator
    src/obj_generator.cpp 
)
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES}
    )
