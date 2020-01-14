cmake_minimum_required(VERSION 3.13)

# create your C++ project
project(FES_EXO VERSION 0.1.0 LANGUAGES CXX)

set(EIGEN_ROOT "C:/dev/eigen" CACHE FILEPATH "Absolute path to Eigen installation.")

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
