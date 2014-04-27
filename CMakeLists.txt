cmake_minimum_required(VERSION 2.6)

# Set the project name
SET(PROJECTNAME stochastic_planner)

# Initialize the project
project(${PROJECTNAME})

# Tried with FindJsonCpp. For the moment could not get it working. Will fix later
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
# Find the JSON CPP package used for configuration
# FIND_PACKAGE(JsonCpp)
add_definitions(-DJSON_IS_AMALGAMATION)

# Find the yarp packages
FIND_PACKAGE(YARP)

# Find the iCub Packages
FIND_PACKAGE(ICUB)

# add include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} include jsoncpp-src-0.6.0-rc2)

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")      

# add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
add_executable(${PROJECTNAME} ./jsoncpp-src-0.6.0-rc2/jsoncpp.cpp ./src/Configuration.cpp ./src/armcontroller.cpp ./src/main.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})