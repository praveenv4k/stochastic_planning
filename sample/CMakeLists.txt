cmake_minimum_required(VERSION 2.8)

PROJECT(HELLO_ICUB)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

find_package(PCL 1.2 REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# add_executable (octree_search octree_search.cpp)
# target_link_libraries (octree_search ${PCL_LIBRARIES})

# add include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")      

ADD_EXECUTABLE(firstproject FirstProject.cpp)

ADD_EXECUTABLE(sampling Sampling.cpp)

ADD_EXECUTABLE(cgalmesh cgalmesh.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(firstproject ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
target_link_libraries (sampling ${PCL_LIBRARIES})