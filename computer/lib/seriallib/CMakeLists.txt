add_library(seriallib STATIC serialib.cpp)

target_include_directories(seriallib PUBLIC
${CMAKE_CURRENT_SOURCE_DIR})
