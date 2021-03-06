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
                                        pcl_conversions pcl_ros
                                        tf)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
#find_package(OpenCV 3.1.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(YAML-CPP REQUIRED)
set(Yaml_Cpp_LIB_DIR /usr/local/lib )
set(Yaml_Cpp_INCLUDE_DIR /usr/local/include )

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR} )
include(/usr/share/OpenCV/OpenCVConfig.cmake)
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

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
    message(STATUS "dir='${dir}'")
endforeach()
message(opencv_lib = ${OpenCV_LIBS})

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS ../..
        CATKIN_DEPENDS roscpp tf
)

#add_library(ikfast_solver src/Ikfast_solver/ikfast_solver.cpp)
#add_dependencies( ${PROJECT_NAME}_gencfg)

#link_directories(/opt/)
add_library(cxy_lsdslam_lib src/tracker/Tracker.cpp src/tracker/Tracker.h
                            src/DataStructure/Frame.cpp src/DataStructure/Frame.h
                            src/utility/ImageHelper.cpp src/utility/ImageHelper.h
                            src/utility/ParameterServer.cpp src/utility/ParameterServer.h
                            src/DataStructure/Frame.cpp src/DataStructure/Frame.h
                            src/utility/DebugUtility.cpp  src/utility/DebugUtility.h
                            src/tracker/TrackRefFrame.cpp src/tracker/TrackRefFrame.h
                            src/utility/MemoryManager.cpp src/utility/MemoryManager.h src/utility/MacroUtility.h
                            src/Optimization/NormalEquationLeastSquare.cpp src/Optimization/NormalEquationLeastSquare.h
                            src/Optimization/OptimizedSelfAdjointMatrix66f.cpp src/Optimization/OptimizedSelfAdjointMatrix66f.h
                            src/DataStructure/DataTypeDeclearation.h
                            src/DataStructure/FramePoseStruct.cpp src/DataStructure/FramePoseStruct.h
                            src/utility/easylogging++.h
                            src/utility/LogUtility.h src/utility/LogUtility.cpp

        )

target_link_libraries(cxy_lsdslam_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})

add_executable(cxy_lsd_main src/main.cpp)
target_link_libraries(cxy_lsd_main  cxy_lsdslam_lib ${YAML_CPP_LIBRARY}
                            ${catkin_LIBRARIES}  ${OpenCV_LIBS})

