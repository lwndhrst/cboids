cmake_minimum_required(VERSION 3.0)
project(cboids C)
set(CMAKE_C_STANDARD 99)
set(CMAKE_GENERATOR "Unix Makefiles")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
    -fopenmp
    $<$<CONFIG:RELEASE>:-O2>
)

find_package(OpenMP REQUIRED)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
add_subdirectory(vendor/raylib)

add_executable(${PROJECT_NAME} src/main.c)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/vendor/raygui/src)
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)
target_link_libraries(${PROJECT_NAME} PRIVATE libomp)
