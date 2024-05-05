find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
  # Update submodules as needed
  option(GIT_SUBMODULE "Check submodules during build" ON)
  if(GIT_SUBMODULE)
    message(STATUS "Submodule update")
    execute_process(
            COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE GIT_SUBMOD_RESULT
        )
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
      message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
  endif()
endif()

find_package(SDL3)
find_package(fmt)
find_package(OpenCV)

if(NOT SDL_FOUND)
  message(STATUS "SDL3 not found, building from source")
  add_subdirectory(sdl)
else()
  message(STATUS "sdl found via find_package")
endif()

if(NOT OpenCV_FOUND)
  message(STATUS "OpenCV not found, building from source")
  add_subdirectory(opencv)
else()
  message(STATUS "OpenCV found via find_package")
endif()

if(NOT fmt_FOUND)
  message(STATUS "fmt not found, building from source")
  add_subdirectory(fmt)
else()
  message(STATUS "fmt found via find_package")
endif()

add_subdirectory(seriallib)


add_library(extern INTERFACE)
target_link_libraries(extern INTERFACE
SDL3::SDL3
fmt::fmt
${OpenCV_LIBS}
seriallib
)
