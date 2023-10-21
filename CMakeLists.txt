cmake_minimum_required(VERSION 3.1.2)
project(compton_cone_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  message_generation
  message_runtime
  mrs_lib
  mrs_msgs
  nodelet
  rad_msgs
  rad_utils
  roscpp
  rospy
  sensor_msgs
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

generate_dynamic_reconfigure_options(
  cfg/compton_cone_generator.cfg
  )

catkin_package(
   CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
   LIBRARIES ComptonConeGenerator ConeAggregator
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

add_library(ConeAggregator src/cone_aggregator.cpp)

target_link_libraries(ConeAggregator
  ${catkin_LIBRARIES}
  )

add_dependencies(ConeAggregator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

