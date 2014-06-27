# project(policysim)
# 
# add_executable(policysim main.cpp)
# 

cmake_minimum_required(VERSION 2.6)

# Set the project name
SET(PROJECTNAME policysim)

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

# MACRO(HEADER_DIRECTORIES return_list)
#     FILE(GLOB_RECURSE new_list *.h)
#     SET(dir_list "")
#     FOREACH(file_path ${new_list})
#         GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
#         SET(dir_list ${dir_list} ${dir_path})
#     ENDFOREACH()
#     LIST(REMOVE_DUPLICATES dir_list)
#     SET(${return_list} ${dir_list})
# ENDMACRO()

# add include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} 
		     include
                    include/appl-0.96/Utils
                    include/appl-0.96/PolicyGraph
                    include/appl-0.96/OfflineSolver
                    include/appl-0.96/Parser
                    include/appl-0.96/Parser/POMDPX
                    include/appl-0.96/Parser/Cassandra
                    include/appl-0.96/Parser/Cassandra/include
                    include/appl-0.96/Models/MOMDP
                    include/appl-0.96/MathLib
                    include/appl-0.96/Evaluator
                    include/appl-0.96/Core
                    include/appl-0.96/Bounds
                    include/appl-0.96/Algorithms
                    include/appl-0.96/Algorithms/SARSOP)

# link_directories(./lib)
# 
# SET(libs appl)

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")      

#add_executable(policysim src/Simulator.cpp src/SimulationEngine.cpp)
add_executable(policysim src/planner.cpp src/StochasticPlannerMain.cpp)

# we now add the YARP and iCub libraries to our project.
#TARGET_LINK_LIBRARIES(policysim ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${CMAKE_SOURCE_DIR}/lib/libappl.a)
TARGET_LINK_LIBRARIES(policysim ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${CMAKE_SOURCE_DIR}/lib/libappl.a)

# Copy the config files to the binary directory
file(GLOB ConfigFiles ${CMAKE_SOURCE_DIR}/config/*.xml
                      ${CMAKE_SOURCE_DIR}/config/*.conf
                      ${CMAKE_SOURCE_DIR}/config/*.json
                      ${CMAKE_SOURCE_DIR}/test/*.*)

                      
foreach(ConfigFile ${ConfigFiles})
  add_custom_command(TARGET StochasticPlanner POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         copy ${ConfigFile} $<TARGET_FILE_DIR:StochasticPlanner>)
endforeach()

#TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
