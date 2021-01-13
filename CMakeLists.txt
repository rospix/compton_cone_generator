cmake_minimum_required(VERSION 3.1.2)
project(compton_cone_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nodelet
  cmake_modules
  rad_msgs
  mrs_msgs
  radiation_utils
  mrs_lib
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/compton_cone_generator.cfg
  )

catkin_package(
   CATKIN_DEPENDS nodelet rospy std_msgs message_runtime rad_msgs mrs_lib radiation_utils mrs_msgs
   LIBRARIES ComptonConeGenerator
   DEPENDS Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_library(ComptonConeGenerator src/compton_cone_generator.cpp)

target_link_libraries(ComptonConeGenerator
  ${catkin_LIBRARIES}
  )

add_dependencies(ComptonConeGenerator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

