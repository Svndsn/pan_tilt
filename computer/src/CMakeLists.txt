add_executable(tracking
main.cpp
Controller.cpp
AngleHandler.cpp
SensorFusion.cpp
Vision.cpp
)
target_include_directories(tracking PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(tracking PRIVATE extern)
target_compile_options(tracking PRIVATE -Wall -Wextra -pedantic -Werror)
