add_executable(vision main.cpp)
target_link_libraries(vision PRIVATE extern)
target_compile_options(vision PRIVATE -Wall -Wextra -pedantic -Werror)
