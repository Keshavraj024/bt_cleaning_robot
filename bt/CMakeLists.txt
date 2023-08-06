cmake_minimum_required(VERSION 3.5)

project(throw_dirt_into_bin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(${PROJECT_NAME} bt.cpp)
target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp_v3)