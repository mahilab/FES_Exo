set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.13)

# create your C++ project
project(mahi-fesexo VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs) # defines conventional GNU isntallation directories

if("${PROJECT_NAME}" STREQUAL "${CMAKE_PROJECT_NAME}")
    option(FES_EXO_EXAMPLES "Builds the example executables" ON)
else()
    option(FES_EXO_EXAMPLES "Builds the example executables" OFF)
endif()

# set compiler flags
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    if (UNIX)
        set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DLinux -D_GNU_SOURCE=1 -std=c++11")
    endif()
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3") # ALL WARNINGS
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /MP")    # MULTICORE BUILDS
endif()

#include
include(FetchContent)

# MAHI DAQ
FetchContent_Declare(mahi-fes GIT_REPOSITORY https://github.com/mahilab/mahi-fes.git)
FetchContent_MakeAvailable(mahi-fes)

FetchContent_Declare(meii 
                     GIT_REPOSITORY https://github.com/mahilab/MEII.git
                     GIT_TAG        origin/main)
FetchContent_MakeAvailable(meii)

#===============================================================================
# CREATE LIBRARY
#===============================================================================

add_library(fesexo "")
add_library(mahi::fesexo ALIAS fesexo)
set_target_properties(fesexo PROPERTIES DEBUG_POSTFIX -d)
target_compile_features(fesexo PUBLIC cxx_std_11)
install(TARGETS fesexo EXPORT mahi-fesexo-targets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
set_target_properties(fesexo PROPERTIES OUTPUT_NAME "mahi-fesexo")

# defines
target_compile_definitions(fesexo PUBLIC FESEXO) # for compatibility checks

# add source files
add_subdirectory(src/Mahi/FesExo)

# add include files
file(GLOB_RECURSE MAHI_FESEXO_HEADERS "include/*.hpp")
target_sources(fesexo PRIVATE ${MAHI_FESEXO_HEADERS}) # for intellisense
target_include_directories(fesexo
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${JSON_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CXXOPTS_INCLUDE_DIR}>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

include_directories(include)

set(FES_EXO_SRC
    src/FESExo/FesGprModel.cpp
    src/FESExo/SharedController.cpp
    src/FESExo/RCModel.cpp
    src/FESExo/MuscleData.cpp
    src/FESExo/Utility.cpp)

if(FES_EXO_EXAMPLES)
    add_subdirectory(src)
endif()