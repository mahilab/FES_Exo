cmake_minimum_required(VERSION 3.13)

# create your C++ project
project(FES_EXO VERSION 0.1.0 LANGUAGES CXX)

# Set output directory to the bin folder instead of in the build folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# find MEL::MEL and all available MEL::xxx modules
find_package(MEII REQUIRED)

# create an app from your source files
add_executable(eating_task "src/eating_task.cpp")

target_link_libraries(eating_task MEII)

target_include_directories(eating_task PRIVATE "include")