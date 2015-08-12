# ## check c++11 / c++0x
# include(CheckCXXCompilerFlag)
# check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
# check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_COMPILER_ARG1 -std=c++0x)
# else()
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)



cmake_minimum_required(VERSION 2.8.3)
project(hanp_local_planner)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  hanp_msgs
  nav_core
  nav_msgs
  pluginlib
  pcl_conversions
  roscpp
  tf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

## install python modules and global scripts
# catkin_python_setup()

## add message, serviece and action files

# add_message_files( DIRECTORY msg
#   FILES
#     Message1.msg
# )

# add_service_files( DIRECTORY srv
#   FILES
#     Service1.srv
# )

# add_action_files( DIRECTORY action
#   FILES
#     Action1.action
# )

## generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs
# )

# add dynamic reconfigure configs
generate_dynamic_reconfigure_options(
  cfg/HANPLocalPlanner.cfg
)

## declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    hanp_local_planner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    hanp_msgs
    nav_core
    nav_msgs
    pluginlib
    pcl_conversions
    roscpp
    tf
    visualization_msgs
#   DEPENDS
#     system_lib
)



## build ##

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

# declare a c++ library
add_library(hanp_local_planner
  src/hanp_local_planner.cpp
  src/context_cost_function.cpp
)

# cmake target dependencies of the c++ library
add_dependencies(hanp_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ library against
target_link_libraries(hanp_local_planner ${catkin_LIBRARIES})



## install ##

## executable scripts (python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## executables and/or libraries for installation
# install(TARGETS hanp_local_planner hanp_local_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## cpp-header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## other files for installation (launch, data, bag files)
# foreach(dir launch data bags)
#    install(DIRECTORY ${dir}/
#       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

## other files for installation
# install(
#   FILES
#     myfile1
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



## testing ##

## add catch based cpp test target
# catkin_add_catchtest(${PROJECT_NAME}-test test/test_hanp_local_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## add nosetest file folders
# catkin_add_nosetests(test)
