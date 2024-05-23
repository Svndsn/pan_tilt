cmake_minimum_required(VERSION 3.20)
project(Target_System
DESCRIPTION "Tracking system for pan-tilt robot."
LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(lib)
add_subdirectory(src)
