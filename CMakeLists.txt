cmake_minimum_required(VERSION 2.8.3)
project(pc_transform_cuda)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(PCL)
find_package(CUDA)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

cuda_add_executable(pc_transform_cuda
  src/pc_transform_cuda.cpp
  src/cuda_process.cu
)
target_link_libraries(pc_transform_cuda
  ${catkin_LIBRARIES}
  ${CUDA_LIBRARIES}
)

cuda_add_executable(pc_transform_pcl
  src/pc_transform_pcl.cpp
)
target_link_libraries(pc_transform_pcl
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
