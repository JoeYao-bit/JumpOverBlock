cmake_minimum_required(VERSION 2.8)

project(JumpOverBlock)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
enable_testing()
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

add_subdirectory(freeNav-base)

add_executable(test_3d_block_viewer test/test_3d_block_viewer.cpp)
target_link_libraries(test_3d_block_viewer freeNav-base)

add_executable(test_los_for_sparse_map test/test_los_for_sparse_map.cpp)
target_link_libraries(test_los_for_sparse_map freeNav-base)

add_executable(test_massive_los_check_with_jump_block test/test_massive_los_check_with_jump_block.cpp)
target_link_libraries(test_massive_los_check_with_jump_block freeNav-base)# ${CGAL_LIBRARIES})
## add ${CGAL_LIBRARIES} to third party lib cause /usr/local/include/g2o/core/base_fixed_sized_edge.hpp:168:33: error: ‘(1.0e+0 / 2.0000000000000001e-9)’ is not a constant expression
#
add_executable(test_detect_best_block_width test/test_detect_best_block_width.cpp)
target_link_libraries(test_detect_best_block_width freeNav-base)