cmake_minimum_required(VERSION 3.0)
project(multilingual-boids-c C)
set(CMAKE_C_STANDARD 99)
set(CMAKE_GENERATOR "Unix Makefiles")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
    -fopenmp
    $<$<CONFIG:RELEASE>:-O2>
)

# add raylib
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # don't build the supplied examples
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE) # don't build the supplied example games
add_subdirectory(raylib)

# declare project executable
add_executable(${PROJECT_NAME} main.c)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/raygui/src)
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)

find_package(OpenMP REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE libomp)
