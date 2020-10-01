set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

include_directories(include)

set(FES_EXO_SRC
    src/FESExo/FesGprModel.cpp
    src/FESExo/SharedController.cpp
    src/FESExo/RCModel.cpp
    include/FESExo/FesGprModel.hpp
    include/FESExo/SharedController.hpp
    include/FESExo/RCModel.hpp
    include/FESExo/Kernels.hpp)

add_executable(fes_calibration src/fes_calibration.cpp)
target_link_libraries(fes_calibration meii::meii mahi::fes)

add_executable(recruitment_curve src/recruitment_curve.cpp icon/fes_exo_icon.rc)
target_link_libraries(recruitment_curve meii::meii mahi::fes)

add_executable(fes_max_recording src/fes_max_recording.cpp)
target_link_libraries(fes_max_recording mahi::fes)

add_executable(fes_max_recording_exo src/fes_max_recording_exo.cpp)
target_link_libraries(fes_max_recording_exo meii::meii mahi::fes)

add_executable(test src/test.cpp ${FES_EXO_SRC})
target_link_libraries(test meii::meii)

add_executable(single_dof_poc src/single_dof_poc.cpp ${FES_EXO_SRC})
target_link_libraries(single_dof_poc meii::meii mahi::fes)

add_executable(multi_dof_poc src/multi_dof_poc.cpp ${FES_EXO_SRC})
target_link_libraries(multi_dof_poc meii::meii mahi::fes)
