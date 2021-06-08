cmake_minimum_required(VERSION 3.19)
project(collision_detection)

set(CMAKE_CXX_STANDARD 14)

###########
## Eigen ##
###########
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})
###########
## CCD ##
###########
find_package(CCD REQUIRED)
include_directories(${CCD_INCLUDE_DIRS})
link_directories(${CCD_LIBRARY_DIRS})
#########
## FCL ##
#########
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(FCL REQUIRED)
include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS})
#########
## OCTO ##
#########
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
#########
## PCL ##
#########
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
###########
## Build ##
###########
add_executable(collision_detection main.cpp octree_viewer.cpp)
target_link_libraries(collision_detection ${FCL_LIBRARIES} ${EIGEN3_LIBRARIES} ${CCD_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(octree_viewer octree_viewer.cpp)
target_link_libraries(octree_viewer ${PCL_LIBRARIES})

add_executable(mesh_box mesh_box.cpp)
target_link_libraries(mesh_box ${FCL_LIBRARIES} ${EIGEN3_LIBRARIES} ${CCD_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(box_box box_box.cpp)
target_link_libraries(box_box ${FCL_LIBRARIES} ${EIGEN3_LIBRARIES} ${CCD_LIBRARIES}  ${PCL_LIBRARIES})