cmake_minimum_required(VERSION 3.13)

# create your C++ project
project(FES_EXO VERSION 0.1.0 LANGUAGES CXX)

# find the eigen path to work with MEII. This shouldn't have to be done, but is for now
set(EIGEN_ROOT "C:/dev/eigen" CACHE FILEPATH "Absolute path to Eigen installation.")
# set(Eigen3_DIR "${PROJECT_SOURCE_DIR}/cmake")
# include the cmake directory
# set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
# set(EIGEN3_INCLUDE_DIR "C:/dev/eigen")
# find_package(Eigen3 REQUIRED)

# Set output directory to the bin folder instead of in the build folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# find MEL::MEL and all available MEL::xxx modules
find_package(MEII REQUIRED)

# add your include directories
include_directories("include" ${EIGEN_ROOT})

# create an app from your source files
add_executable(eating_task
    "src/eating_task.cpp"
)

target_link_libraries(eating_task 
    MEII
)
