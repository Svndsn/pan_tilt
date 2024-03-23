add_executable(controller main.cpp)
target_link_libraries(controller PRIVATE extern)
target_compile_options(controller PRIVATE -Wall -Wextra -pedantic -Werror)
