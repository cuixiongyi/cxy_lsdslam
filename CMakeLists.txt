cmake_minimum_required(VERSION 2.8.12)
project(cxy_lsdslam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#include_directories($ROS_PACKAGE_PATH)
set(CMAKE_INSTALL_PREFIX ~/workspace/install)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        roslib
                                        std_msgs
                                        genmsg
                                        dynamic_reconfigure
                                        visualization_msgs
                                        sensor_msgs
                                        tf)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
find_package(OpenCV REQUIRED)
set(Yaml_Cpp_LIB_DIR /usr/local/lib )
set(Yaml_Cpp_INCLUDE_DIR /usr/local/include )
include_directories(Yaml_Cpp_INCLUDE_DIR )
include_directories(EIGEN3_INCLUDE_DIR )
#find_package()
#
#generate_dynamic_reconfigure_options(
#        cfg/cxy_lsdslam.cfg
#)


include_directories(
        include
        ${EIGEN_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
        ${catkin_INCLUDE_DIRS}
        ${Yaml_Cpp_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS ../..
        CATKIN_DEPENDS roscpp tf
)

#add_library(ikfast_solver src/Ikfast_solver/ikfast_solver.cpp)
#add_dependencies( ${PROJECT_NAME}_gencfg)

link_directories(/opt/)
add_executable(cxy_lsd_main src/main.cpp)
target_link_libraries(cxy_lsd_main  ${catkin_LIBRARIES} /usr/local/lib/libyaml-cpp.so ${OpenCV_LIBS})

