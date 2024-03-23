cmake_minimum_required(VERSION 3.20)
project(Target_System
DESCRIPTION "Targeting system for pan-tilt robot."
LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Build parallel
set(CMAKE_BUILD_PARALLEL_LEVEL 10)

add_subdirectory(lib)
add_subdirectory(src)

