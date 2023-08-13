cmake_minimum_required(VERSION 3.10)
project(cbf_msgs)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        visualization_msgs
        )

add_message_files(
        DIRECTORY msg
        FILES
        Polytopes.msg
        Obstacle.msg
        Distances.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        visualization_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        visualization_msgs
)
