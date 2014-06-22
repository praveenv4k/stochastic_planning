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

find_library(CGAL_LIBRARY CGAL /usr/local)

# add include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} include jsoncpp-src-0.6.0-rc2)


# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")      

# add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
# add_executable(${PROJECTNAME} ./jsoncpp-src-0.6.0-rc2/jsoncpp.cpp ./src/Configuration.cpp ./src/armcontroller.cpp src/StoachasticPlannerMain.cpp)
add_executable(ObjectController src/objectcontroller.cpp src/ObjectControllerMain.cpp)
add_executable(CartesianControl jsoncpp-src-0.6.0-rc2/jsoncpp.cpp src/Config.cpp src/CartesianControl.cpp src/grasp.cpp src/armcontrol.cpp)
add_executable(StochasicPlanner src/StochasticPlannerMain.cpp src/planner.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(StochasicPlanner ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(CartesianControl ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${CGAL_LIBRARY})
TARGET_LINK_LIBRARIES(ObjectController ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

# Copy the config files to the binary directory
file(GLOB ConfigFiles ${CMAKE_SOURCE_DIR}/config/*.xml
                      ${CMAKE_SOURCE_DIR}/config/*.conf
                      ${CMAKE_SOURCE_DIR}/config/*.json)
                   
foreach(ConfigFile ${ConfigFiles})
  add_custom_command(TARGET CartesianControl POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         copy ${ConfigFile} $<TARGET_FILE_DIR:CartesianControl>)
endforeach()
#TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})