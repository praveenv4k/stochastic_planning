cmake_minimum_required(VERSION 2.6)

# Set the project name
SET(PROJECTNAME stochastic_planning)

# Initialize the project
project(${PROJECTNAME})

add_definitions(-DJSON_IS_AMALGAMATION)

# add include directories
# Find the yarp packages
FIND_PACKAGE(YARP)

# Find the iCub Packages
FIND_PACKAGE(ICUB)

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

# add include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} 
                    include/common
                    include/domain_generator
                    include/agent_controller
                    include/policy_simulator
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
                    include/appl-0.96/Algorithms/SARSOP
                    include/jsoncpp-src-0.6.0-rc2)

add_executable(domain_generator 
               src/jsoncpp-src-0.6.0-rc2/jsoncpp.cpp 
               src/common/Config.cpp 
               src/domain_generator/AbstractDDLGenerator.cpp 
               src/domain_generator/POMDPFileGenerator.cpp 
               src/domain_generator/DomainExtractor.cpp 
               src/domain_generator/main.cpp)

add_executable(robot_controller 
               src/jsoncpp-src-0.6.0-rc2/jsoncpp.cpp 
               src/common/Config.cpp 
               src/agent_controller/CartesianControl.cpp 
               src/agent_controller/armcontrol.cpp)

add_executable(object_controller 
               src/jsoncpp-src-0.6.0-rc2/jsoncpp.cpp 
               src/common/Config.cpp 
               src/agent_controller/objectcontroller.cpp 
               src/agent_controller/ObjectControllerMain.cpp)

add_executable(policy_simulator 
               src/jsoncpp-src-0.6.0-rc2/jsoncpp.cpp 
               src/common/Config.cpp 
               src/policy_simulator/PolicySimulator.cpp 
               src/policy_simulator/PolicySimulatorMain.cpp)

TARGET_LINK_LIBRARIES(robot_controller ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(object_controller ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(policy_simulator ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${CMAKE_SOURCE_DIR}/lib/libappl.a)
