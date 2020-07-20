cmake_minimum_required(VERSION 3.13)

# create your C++ project
project(fes_exo VERSION 0.1.0 LANGUAGES CXX)

#include
include(FetchContent)

# MAHI DAQ
FetchContent_Declare(mahi-fes GIT_REPOSITORY https://github.com/mahilab/mahi-fes.git)
FetchContent_MakeAvailable(mahi-fes)

FetchContent_Declare(meii GIT_REPOSITORY https://github.com/mahilab/MEII.git)
FetchContent_MakeAvailable(meii)

# create executables
# add_executable(eating_task "src/eating_task.cpp")
# target_link_libraries(eating_task meii::meii)

add_executable(fes_calibration src/fes_calibration.cpp)
target_link_libraries(fes_calibration meii::meii mahi::fes)

add_executable(fes_max_recording src/fes_max_recording.cpp)
target_link_libraries(fes_max_recording mahi::fes)

add_executable(test src/test.cpp)
target_link_libraries(test meii::meii)