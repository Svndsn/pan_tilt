add_executable(targeting
main.cpp
Controller.cpp
AngleHandler.cpp
SensorFusion.cpp
)
target_link_libraries(targeting PRIVATE extern)
target_compile_options(targeting PRIVATE -Wall -Wextra -pedantic -Werror)
