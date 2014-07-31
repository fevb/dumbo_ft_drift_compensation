cmake_minimum_required(VERSION 2.8.3)
project(dumbo_ft_drift_compensation)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs iir_filter std_srvs eigen_conversions)
find_package(Eigen REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  DEPENDS eigen
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs iir_filter std_srvs eigen_conversions
  INCLUDE_DIRS include 
  LIBRARIES 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_executable(ft_drift_compensation_node src/ft_drift_compensation_node.cpp src/ft_drift_compensation.cpp src/ft_drift_compensation_params.cpp)
target_link_libraries(ft_drift_compensation_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ft_drift_compensation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)