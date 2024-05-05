add_executable(tracking
main.cpp
Controller.cpp
AngleHandler.cpp
SensorFusion.cpp
Vision.cpp
)
target_link_libraries(tracking PRIVATE extern)
target_compile_options(tracking PRIVATE -Wall -Wextra -pedantic -Werror)
