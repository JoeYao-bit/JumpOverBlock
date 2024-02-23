cmake_minimum_required(VERSION 2.8)

project(freeNav)
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

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)
find_package(Glog REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Octomap REQUIRED)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem thread graph)
#find_package(OMPL REQUIRED)
find_package(LibXml2)
find_package(argparse)

if( NOT LIBXML2_FOUND )
    message( FATAL_ERROR "libxml2 not found\n" )
    return()
endif()

find_package(Python3 COMPONENTS Interpreter Development)

message ( STATUS "Python3_FOUND = ${Python3_FOUND} " )
message ( STATUS "Python3_INCLUDE_DIRS = ${Python3_INCLUDE_DIRS} " )
message ( STATUS "Python3_LIBRARIES = ${Python3_LIBRARIES} " )

# 开启QT用于预处理的组件
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找QT的模块
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Charts REQUIRED)

set(QT_LIBS
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::PrintSupport
        Qt5::Charts
        )

add_subdirectory(third_party/grid-pathfinding/graph ./graph)

INCLUDE_DIRECTORIES(${Qt5Core_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Gui_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5OpenGL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5PrintSupport_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Charts_INCLUDE_DIRS})

include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${argparse_INCLUDE_DIRS})

if(POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE "GLVND")
endif()
#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
find_package(Octomap REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

option(ENABLE_LOG "whether enable log" ON)
IF(ENABLE_LOG)
    message(STATUS "Enable log print for debug" )
    add_definitions( -DENABLE_LOG)
ENDIF(ENABLE_LOG)

include_directories(
  SYSTEM
  ${CSPARSE_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OCTOMAP_DIRS}
)

include(GoogleTest)


include_directories(${EIGEN3_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIR}
                    gtest
                    ${GLOG_INCLUDE_DIRS}
                    core
                    global_planner
                    motion_planner
                    motion_model
                    environment
                    test/
                    third_party/ENL-SVG/Pathfinding
                    third_party/move_base_2d
                    third_party/ompl_path_planning
                    third_party/jps_2d_3d/include
                    third_party/lapinozz_2d
                    third_party/3D-AStar-ThetaStar-master
                    third_party/
                    third_party/DOSL
                    third_party/DynamicVoronoi2D
                    third_party/dynamicvoronoi
                    third_party/ros_motion_planner
                    third_party/TopologyPathPlanning/include
        )

set(G2O_LIBS
        g2o_cli
        g2o_ext_freeglut_minimal
        g2o_simulator
        g2o_solver_slam2d_linear
        g2o_types_icp
        g2o_types_slam2d
        g2o_core g2o_interface
        g2o_solver_csparse
        g2o_solver_structure_only
        g2o_types_sba
        g2o_types_slam3d
        g2o_csparse_extension
        g2o_opengl_helper
        g2o_solver_dense
        g2o_stuff
        g2o_types_sclam2d
        g2o_parser
        g2o_solver_pcg
        g2o_types_data
        g2o_types_sim3
        cxsparse)

aux_source_directory(core                                core_src)
aux_source_directory(motion_planner/time_elastic_band    motion_planner_src)
aux_source_directory(motion_planner/path_smooth          raw_teb_src)
aux_source_directory(motion_model/differential_wheeled   motion_model_src)
aux_source_directory(environment/2d_grid                 2d_environment_src)
aux_source_directory(environment/3d_octomap              3d_environment_src)
aux_source_directory(environment/3d_textmap              3d_textmap_src)
aux_source_directory(test/dependencies                   visualize_src)
aux_source_directory(test/canvas                         canvas_src)
aux_source_directory(test/3d_viewer                      3d_viewer_src)
aux_source_directory(test/massive_scene_loader           massive_scene_loader_src)
aux_source_directory(motion_planner/path_smooth          path_smooth_src)
aux_source_directory(global_planner/rim_jump             rim_jump_src)
aux_source_directory(global_planner/multi-agent-path-finding  mapf_src)
aux_source_directory(global_planner/my_cbs               my_cbs_src)
aux_source_directory(global_planner/layered_mapf cbs_with_time_src)

aux_source_directory(global_planner/my_eecbs             my_eecbs_src)
aux_source_directory(global_planner/second_EECBS         second_eecbs_src)
aux_source_directory(third_party/DynamicVoronoi2D        DynamicVoronoi2D_src)
aux_source_directory(third_party/dynamicvoronoi          dynamicvoronoi_src)

aux_source_directory(third_party/ENL-SVG/src  ENLSVG_src)
aux_source_directory(third_party/move_base_2d            move_base_2d_src)
aux_source_directory(third_party/ompl_path_planning      ompl_path_planning_src)
aux_source_directory(third_party/jps_2d_3d/src/distance_map_planner      jps_src_1)
aux_source_directory(third_party/jps_2d_3d/src/jps_planner      jps_src_2)
aux_source_directory(third_party/lapinozz_2d                lapinozz_2d_src)
aux_source_directory(third_party/3D-AStar-ThetaStar-master  3D-AStar-ThetaStar_src)
aux_source_directory(third_party/EECBS/src                  EECBS_src)
aux_source_directory(third_party/CBSH2-RTC/src              CBSH2-RTC_src)
aux_source_directory(third_party/PBS/src                    PBS_src)
aux_source_directory(third_party/lacam/src                  lacam_src)
aux_source_directory(third_party/lacam2/src                 lacam2_src)
aux_source_directory(third_party/MAPF-LNS2/src              MAPF_LNS2_src)
aux_source_directory(third_party/MAPF-LNS2/src/CBS          MAPF_LNS2_CBS_src)
aux_source_directory(third_party/MAPF-LNS2/src/PIBT         MAPF_LNS2_PIBT_src)
aux_source_directory(third_party/pibt2/src                  PIBT2_src)

aux_source_directory(third_party/ros_motion_planner/global_planner ros_global_src)
aux_source_directory(third_party/ros_motion_planner/graph_planner  ros_graph_src)
aux_source_directory(third_party/ros_motion_planner/sample_planner ros_sample_src)
aux_source_directory(test/qt_gui qt_gui_src)
aux_source_directory(test/teb_log_gui teb_log_qt_gui_src)
aux_source_directory(third_party/TopologyPathPlanning harrt_src)


add_library(ENL-SVG_LIB                ${ENLSVG_src})
add_library(move_base_2d_lib           ${move_base_2d_src})
add_library(ompl_path_planning_src_lib ${ompl_path_planning_src})
add_library(jps_lib                    ${jps_src_1} ${jps_src_2})
add_library(lapinozz_2d_lib            ${lapinozz_2d_src})
add_library(3D-AStar-ThetaStar_lib     ${3D-AStar-ThetaStar_src})
add_library(EECBS_lib                  ${EECBS_src} ${PBS_src} ${lacam_src} ${CBSH2-RTC_src} ${lacam2_src}
                                       ${MAPF_LNS2_src} ${MAPF_LNS2_CBS_src} ${MAPF_LNS2_PIBT_src} ${PIBT2_src})
#add_library(PBS_lib                    ${PBS_src})
#add_library(lacam_lib                  ${lacam_src})
add_library(DynamicVoronoi2D_lib       ${DynamicVoronoi2D_src})
add_library(dynamicvoronoi_lib         ${dynamicvoronoi_src})
add_library(ros_global_planner_lib     ${ros_global_src}
                                       ${ros_graph_src}
                                       ${ros_sample_src})

#add_library(harrt_src_lib ${harrt_src})
#target_link_libraries(harrt_src_lib ${OpenCV_LIBS} ${CGAL_LIBRARIES})

add_library(freeNav ${core_src}
                    ${motion_planner_src}
                    ${motion_model_src}
                    ${2d_environment_src}
                    ${3d_environment_src}
                    ${3d_textmap_src}
                    ${canvas_src}
                    ${visualize_src}
                    ${path_smooth_src}
                    ${raw_teb_src}
                    ${rim_jump_src}
                    ${mapf_src}
                    ${my_cbs_src}
                    ${cbs_with_time_src}
                    ${my_eecbs_src}
                    ${second_eecbs_src}
                    ${3d_viewer_src}
                    ${path_planning_loader_src}
                    ${massive_scene_loader_src}
                    ${move_base_2d_src}
                    #${OMPL_LIBRARY}

        )

set(EXTERNAL_LIBS
        ${QT_LIBS}
        ${SUITESPARSE_LIBRARIES}
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        gtest
        glog
        gtest_main
        pthread
        ${G2O_LIBS}
        ${CSPARSE_LIB}
        ${OCTOMAP_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        ${Pangolin_LIBRARIES}
        #${CGAL_LIBRARIES} # cause /usr/local/include/g2o/core/base_fixed_sized_edge.hpp:168:33: error: ‘(1.0e+0 / 2.0000000000000001e-9)’ is not a constant expression
        ENL-SVG_LIB
        move_base_2d_lib
        ${yaml-cpp}
        /usr/local/lib/libompl.so
        ompl_path_planning_src_lib
        jps_lib
        lapinozz_2d_lib
        3D-AStar-ThetaStar_lib
        EECBS_lib
        #lacam_lib
        #PBS_lib
        ${Boost_LIBRARIES}
        #${LIBXML2}
        #${DSL_LIBRARY}
        DynamicVoronoi2D_lib
        dynamicvoronoi_lib
        ros_global_planner_lib
        LibXml2::LibXml2
        ${Python3_LIBRARIES}
        /usr/local/lib/x86_64-linux-gnu/libkahypar.so
        ${argparse_lib}
        lib-graph
        )

target_link_libraries(freeNav ${EXTERNAL_LIBS})

add_executable(test_curve_fitting test/learn_g2o/test_curve_fitting.cpp)
target_link_libraries(test_curve_fitting freeNav)

add_executable(test_circle_fitting test/learn_g2o/test_circle_fitting.cpp)
target_link_libraries(test_circle_fitting freeNav)

add_executable(test_path_smooth motion_planner/path_smooth/test_path_smooth.cpp)
target_link_libraries(test_path_smooth freeNav)

add_executable(test_canvas test/test_canvas.cpp)
target_link_libraries(test_canvas freeNav)

add_executable(test_rimjump_detect_surface test/rim_jump/test_surface_process.cpp)
target_link_libraries(test_rimjump_detect_surface freeNav)

add_executable(test_tangent_graph_build test/rim_jump/test_2D_path_planning.cpp)
target_link_libraries(test_tangent_graph_build freeNav)# harrt_src_lib)

add_executable(test_3d_viewer test/rim_jump/test_3d_viewer.cpp)
target_link_libraries(test_3d_viewer freeNav)

add_executable(test_3d_block_viewer test/jump_over_block/test_3d_block_viewer.cpp)
target_link_libraries(test_3d_block_viewer freeNav)

add_executable(test_opengl test/test_opengl.cpp)
target_link_libraries(test_opengl freeNav)

add_executable(test_ENLSVG test/test_ENLSVG.cpp)
target_link_libraries(test_ENLSVG freeNav)

add_executable(test_auto_sorted_queue test/test_auto_sorted_queue.cpp)
target_link_libraries(test_auto_sorted_queue freeNav)

add_executable(test_single_path_planning_wrapper test/rim_jump/test_single_path_planning_wrapper.cpp)
target_link_libraries(test_single_path_planning_wrapper freeNav)

add_executable(test_massive_comparison test/rim_jump/test_massive_comparison_2d.cpp)
target_link_libraries(test_massive_comparison freeNav)

add_executable(test_graph_compare test/rim_jump/test_graph_compare.cpp)
target_link_libraries(test_graph_compare freeNav)

add_executable(test_massive_comparison_3d test/rim_jump/test_massive_comparison_3d.cpp)
target_link_libraries(test_massive_comparison_3d freeNav)

add_executable(directory_detect_test test/rim_jump/test_directory_detect.cpp)
target_link_libraries(directory_detect_test freeNav)

add_executable(test_los_for_sparse_map test/jump_over_block/test_los_for_sparse_map.cpp)
target_link_libraries(test_los_for_sparse_map freeNav)

add_executable(test_massive_los_check_with_jump_block test/jump_over_block/test_massive_los_check_with_jump_block.cpp)
target_link_libraries(test_massive_los_check_with_jump_block freeNav)# ${CGAL_LIBRARIES})
# add ${CGAL_LIBRARIES} to third party lib cause /usr/local/include/g2o/core/base_fixed_sized_edge.hpp:168:33: error: ‘(1.0e+0 / 2.0000000000000001e-9)’ is not a constant expression

add_executable(test_detect_best_block_width test/jump_over_block/test_detect_best_block_width.cpp)
target_link_libraries(test_detect_best_block_width freeNav)

add_executable(test_2d_mapf_viewer test/multi_agent_path_finding/test_2d_mapf_viewer.cpp)
target_link_libraries(test_2d_mapf_viewer freeNav)

add_executable(test_massive_layered_mapf test/multi_agent_path_finding/test_massive_layered_mapf.cpp)
target_link_libraries(test_massive_layered_mapf freeNav)

add_executable(test_create_random_map_fr_mapf test/multi_agent_path_finding/test_create_random_map_fr_mapf.cpp)
target_link_libraries(test_create_random_map_fr_mapf freeNav ${EXTERNAL_LIBS})

add_executable(test_distinctive_topology_path_planning test/distinctive_topo/test_dosl_distinctive_topology_path_planning.cpp)
target_link_libraries(test_distinctive_topology_path_planning freeNav)

add_executable(Dynamic_Voronoi_example test/test_Dynamic_Voronoi_example.cpp)
target_link_libraries(Dynamic_Voronoi_example freeNav ${EXTERNAL_LIBS})

add_executable(QT_gui ${qt_gui_src} test/qt_gui/resources/images.qrc)
target_link_libraries(QT_gui freeNav ${EXTERNAL_LIBS})

add_executable(TEB_LOG_QT_GUI ${teb_log_qt_gui_src} test/teb_log_gui/resources/images.qrc)
target_link_libraries(TEB_LOG_QT_GUI freeNav ${EXTERNAL_LIBS})

add_executable(PBS_driver third_party/PBS/src/driver.cpp)
target_link_libraries(PBS_driver freeNav ${EXTERNAL_LIBS})

add_executable(test_lacam test/multi_agent_path_finding/test_lacam.cpp)
target_link_libraries(test_lacam freeNav ${EXTERNAL_LIBS})

add_executable(test_CBSH2_RTC third_party/CBSH2-RTC/src/driver.cpp)
target_link_libraries(test_CBSH2_RTC freeNav ${EXTERNAL_LIBS})

add_executable(test_MAPF_LNS2 third_party/MAPF-LNS2/src/driver.cpp)
target_link_libraries(test_MAPF_LNS2 freeNav ${EXTERNAL_LIBS})

add_executable(test_lacam2 third_party/lacam2/main.cpp)
target_link_libraries(test_lacam2 freeNav ${EXTERNAL_LIBS})

add_executable(test_pibt2_mapf third_party/pibt2/mapf.cpp)
target_link_libraries(test_pibt2_mapf freeNav ${EXTERNAL_LIBS})

add_executable(test_pibt2_mapd third_party/pibt2/mapd.cpp)
target_link_libraries(test_pibt2_mapd freeNav ${EXTERNAL_LIBS})

add_executable(test_memory_check test/multi_agent_path_finding/test_memory_check.cpp)
target_link_libraries(test_memory_check freeNav)