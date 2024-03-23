add_executable(tracking opencv.cpp)
target_link_libraries(tracking PRIVATE extern)
target_compile_options(tracking PRIVATE -Wall -Wextra -pedantic -Werror)
