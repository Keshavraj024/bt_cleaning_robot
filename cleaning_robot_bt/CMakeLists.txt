cmake_minimum_required(VERSION 3.5)

project(throw_dirt_into_bin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp REQUIRED)

add_executable(${PROJECT_NAME} cleaning_robot_bt.cpp)
target_link_libraries(${PROJECT_NAME} behaviortree_cpp)
