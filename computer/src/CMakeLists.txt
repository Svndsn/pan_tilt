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

add_executable(test_setpoint
test-setpoint.cpp
Controller.cpp
AngleHandler.cpp
SensorFusion.cpp
Vision.cpp
)
target_include_directories(test_setpoint PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(test_setpoint PRIVATE extern)
target_compile_options(test_setpoint PRIVATE -Wall -Wextra -pedantic -Werror)

add_executable(test_vision
test-vision.cpp
Controller.cpp
AngleHandler.cpp
SensorFusion.cpp
Vision.cpp
)
target_include_directories(test_vision PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(test_vision PRIVATE extern)
target_compile_options(test_vision PRIVATE -Wall -Wextra -pedantic -Werror)
