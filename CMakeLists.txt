cmake_minimum_required(VERSION 2.8.3)
project(Sn3DCarScan)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(LIBS_OPTIONAL_3RD)
#find OpenCV
set("OpenCV_DIR" "/usr/local/share/OpenCV")
find_package(OpenCV 3.0 REQUIRED)
if(OPENCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_DIR}/lib)
  list(APPEND  LIBS_OPTIONAL_3RD ${OpenCV_LIBRARIES}) 
else()
  message(FATAL_ERROR "can't find opencv !")
endif ()

#find catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  pcl_conversions
  pcl_ros
)

if(catkin_INCLUDE_DIRS)
  include_directories(${catkin_INCLUDE_DIRS})
  list(APPEND  LIBS_OPTIONAL_3RD   ${catkin_LIBRARIES}) 
else()
  message(FATAL_ERROR "can't find catkin !")
endif()

#find qt5
set(Qt5_DIR "/home/extrelin/Qt5.13.1/5.13.1/gcc_64/lib/cmake/Qt5")
find_package(Qt5 5.13  REQUIRED COMPONENTS  Widgets Concurrent )
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
list(APPEND  LIBS_OPTIONAL_3RD    Qt5::Widgets  Qt5::Concurrent) 

#find boost 
find_package(Boost REQUIRED)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    list(APPEND  LIBS_OPTIONAL_3RD      ${Boost_LIBRARIES}) 
else()
    message(FATAL_ERROR "can't find boost !")
endif()

#find cuda
find_package(CUDA 9.0 REQUIRED)
if (CUDA_FOUND)
    include_directories("${CUDA_INCLUDE_DIRS}")
    list(APPEND  LIBS_OPTIONAL_3RD   ${CUDA_LIBRARIES}) 
    # Optional: Specify the arch of your CUDA hardware here
else()
  message(FATAL_ERROR "can't find cuda!")
endif()

#find eigen
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  include_directories("${EIGEN3_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "can't find eigen3!")
endif ()

#find openmp
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  ${OpenMP_CXX_FLAGS}")
else()
  message(FATAL_ERROR "can't find openmp!")
endif()

#find openmesh
find_package( OpenMesh REQUIRED)
if(OPENMESH_FOUND)
  include_directories( ${OpenMesh_INCLUDE_DIRS} )
  list(APPEND  LIBS_OPTIONAL_3RD     ${OpenMesh_LIBS}) 
  list(APPEND  LIBS_OPTIONAL_3RD     OpenMeshCore) 
  list(APPEND  LIBS_OPTIONAL_3RD     OpenMeshTools) 
else()
  message(FATAL_ERROR "can't find openmesh")
endif()


#This project also need glog , flann and ceres but I install in my  ubuntu 18.04 system, so if your system is windows 
#you need to find_package glog and ceres

#find glog
list(APPEND  LIBS_OPTIONAL_3RD  glog) 
#find ceres 
list(APPEND  LIBS_OPTIONAL_3RD   ceres) 
#find flann

# ------------------------------------------------

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Weffc++")

# find all sources files
file(GLOB_RECURSE HEADERS  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB_RECURSE SOURCES  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c*)
file(GLOB_RECURSE CUDAS  ${CMAKE_CURRENT_SOURCE_DIR}/cuda/*.cu)
set(FORMS src/mainwindow.ui)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions HEADERS action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added me and services with any dependencies listed here
# generate_messages(HEADERS
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Sn3DCarScan
#  CATKIN_DEPENDS ros std_msgs
#  DEPENDS system_library
)

###########
## Build ##
###########

#include
include_directories(
   include/sn3d_car_auto_scan
   include/ros_file
   include/fusion
   include/robust_pcl_registration
   include/vulkan_window
   include
)

#cuda compile
CUDA_COMPILE(cuda_Target  ${CUDAS} )
add_executable(${PROJECT_NAME}_node ${HEADERS} ${SOURCES} ${FORMS}  ${cuda_Target} )
set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE ${CUDA_C_OR_CXX})
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} " -gencode arch=compute_20,code=sm_20")

#link libraries
 target_link_libraries(${PROJECT_NAME}_node
    ${LIBS_OPTIONAL_3RD}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2RGB
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_Sn3DCarScan.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
