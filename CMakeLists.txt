cmake_minimum_required(VERSION 3.14)
project(ri_dvp VERSION 1.0.0 LANGUAGES CXX)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Optimization flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Option for OpenMP parallelization
option(USE_OPENMP "Enable OpenMP parallelization" ON)
if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        message(STATUS "OpenMP found, enabling parallelization")
    endif()
endif()

# ============================================================================
# Library
# ============================================================================
add_library(ri_dvp_lib STATIC
    src/ri_dvp_components.cpp
    src/ri_dvp_processor.cpp
)

target_include_directories(ri_dvp_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ri_dvp_lib
    PUBLIC
        Eigen3::Eigen
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(ri_dvp_lib PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(ri_dvp_lib PUBLIC USE_OPENMP)
endif()

# ============================================================================
# Main Executable
# ============================================================================
add_executable(ri_dvp_demo src/main.cpp)
target_link_libraries(ri_dvp_demo PRIVATE ri_dvp_lib)

# ============================================================================
# Installation
# ============================================================================
install(TARGETS ri_dvp_lib ri_dvp_demo
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "========================================")
message(STATUS "RI-DVP Configuration Summary")
message(STATUS "========================================")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenMP: ${OpenMP_CXX_FOUND}")
message(STATUS "========================================")
message(STATUS "")
