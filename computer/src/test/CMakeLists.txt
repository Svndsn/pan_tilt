add_executable(test
main.cpp
SensorFusion.cpp
)
target_link_libraries(test PRIVATE extern)
target_compile_options(test PRIVATE -Wall -Wextra -pedantic)
