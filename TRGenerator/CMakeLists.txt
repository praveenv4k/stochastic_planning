cmake_minimum_required(VERSION 2.6)

project(trgenerator)

#add_subdirectory(include)
#add_subdirectory(src)

# Tried with FindJsonCpp. For the moment could not get it working. Will fix later
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
# Find the JSON CPP package used for configuration
# FIND_PACKAGE(JsonCpp)
add_definitions(-DJSON_IS_AMALGAMATION)

# add include directories
include_directories(include jsoncpp-src-0.6.0-rc2)

add_executable(trgenerator jsoncpp-src-0.6.0-rc2/jsoncpp.cpp src/Config.cpp src/main.cpp)

# Copy the config files to the binary directory
file(GLOB ConfigFiles ${CMAKE_SOURCE_DIR}/config/*.xml
                      ${CMAKE_SOURCE_DIR}/config/*.conf
                      ${CMAKE_SOURCE_DIR}/config/*.json)
foreach(ConfigFile ${ConfigFiles})
  add_custom_command(TARGET trgenerator POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         copy ${ConfigFile} $<TARGET_FILE_DIR:trgenerator>)
endforeach()

# add_custom_command(TARGET trgenerator POST_BUILD
#                    COMMAND ${CMAKE_COMMAND} -E copy_directory
#                        ${CMAKE_SOURCE_DIR}/config $<TARGET_FILE_DIR:trgenerator>)

