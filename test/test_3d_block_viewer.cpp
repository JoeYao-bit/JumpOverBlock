//
// Created by yaozhuo on 2022/2/26.
//


#include "octomap/octomap.h"
#include <sys/time.h>

#include "../freeNav-base/dependencies/color_table.h"
#include "../freeNav-base/dependencies/random_map_generator.h"
#include "../freeNav-base/dependencies/3d_textmap/voxel_loader.h"
#include "../freeNav-base/dependencies/thread_pool.h"
#include "../freeNav-base/dependencies/massive_scene_loader/ScenarioLoader3D.h"
#include "../freeNav-base/basic_elements/surface_process.h"
#include "../freeNav-base/test/test_data.h"

#include "../freeNav-base/visualization/3d_viewer/3d_viewer.h"

#include "../algorithm/block_detect.h"
#include "../algorithm/line_of_sight_jump_between_block.h"
#include "../algorithm/surface_process_jump_block.h"

ThreadPool viewer_thread(1);

using namespace freeNav;
using namespace freeNav::JOB;

Viewer3DBase* viewer_3d;

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

#define OCTO_MAP 0
#if OCTO_MAP
OctoMapLoader* oml_ = nullptr;
#else
TextMapLoader_3D* oml_ = nullptr;
#endif



bool plan_finish = false;

Pointi<3> pt1, pt2;
GridPtr<3> sg1 = std::make_shared<Grid<3>>(), sg2 = std::make_shared<Grid<3>>();


std::vector<Path<3> > paths;
Path<3> path_rrt, path_jps, path_astar;

// MapTestConfig_Full4 // run out of space, shrink space 4
// MapTestConfig_Simple // success
// MapTestConfig_DA2
// MapTestConfig_DC1 // one bywave, need 8~14s
// MapTestConfig_Complex
// MapTestConfig_A1
// MapTestConfig_FA2
// MapTestConfig_A5
// MapTestConfig_EB2
auto config = MapTestConfig_A1;

std::string file_path = "/home/yaozhuo/code/free-nav/resource/binary/fr_campus.vis";
std::string map_path = config.at("map_path");

TextMapLoader_3D* oml_shrink = nullptr;

GridPtrs<3> visible_points_start;
GridPtrs<3> visible_points_target;

int main() {
#if OCTO_MAP
    oml_ = new OctoMapLoader("/home/yaozhuo/code/free-nav/resource/map/fr_campus.bt", 6);
#else
    oml_ = new TextMapLoader_3D(map_path, 1 //atoi(config.at("shrink_level").c_str())
    ); // shrink to enable load

//    oml_shrink = new TextMapLoader_3D(map_path, atoi(config.at("shrink_level").c_str()));

    file_path = config.at("vis_path");
#endif
    // init random map
    //RandomMapGenerator<3> random_map(oml_->getDimensionInfo(), 20, 5, file_path, false);
    auto& random_map_config = grid_3D_3;
    RandomMapGenerator<3> random_map(random_map_config.dim_,
                                     random_map_config.cubic_half_width_,
                                     random_map_config.cubic_number_,
                                     random_map_config.random_file_path_,
                                     false);

    auto is_occupied_random = [&](const Pointi<3> & pt) -> bool {
        if(isOutOfBoundary(pt, random_map.dimen_)) { return true; }
        Id id = PointiToId(pt, random_map.dimen_);
        return random_map.occ_ids_.find(id) != random_map.occ_ids_.end();
    };


    // load octomap
    auto is_occupied = [](const Pointi<3> & pt) -> bool { return oml_->isOccupied(pt); };
    auto set_occupied = [](const Pointi<3> & pt) { oml_->setOccupied(pt); };
    IS_OCCUPIED_FUNC<3> is_occupied_func = is_occupied;
    SET_OCCUPIED_FUNC<3> set_occupied_func = set_occupied;


    gettimeofday(&tv_pre, &tz);
//    auto surface_processor_shrink = std::make_shared<SurfaceProcessorSparseWithJumpBlockGreedy<3> >(oml_shrink->getDimensionInfo(),
//                                                                                      is_occupied_func_shrink,
//                                                                                      set_occupied_func_shrink,
//                                                                                      oml_shrink->occ_voxels_,
//                                                                                      oml_shrink->occ_voxel_ids_,
//                                                                                       2,//atoi(config.at("minimum_block_width").c_str()),
//                                                                                       "",
//                                                                                       true);

    auto surface_processor =
            std::make_shared<SurfaceProcessorSparseWithJumpBlockGreedyShrink<3> >(oml_->getDimensionInfo(),
                                                                                 is_occupied_func,
                                                                                 set_occupied_func,
                                                                                 oml_->occ_voxels_,
                                                                                 oml_->occ_voxel_ids_,
                                                                                 atoi(config.at("shrink_level").c_str()),
                                                                                 atof(config.at("minimum_block_width").c_str()),
                                                                                 config.at("block_path").c_str(),
                                                                                 true);

//    auto surface_processor =
//            std::make_shared<SurfaceProcessorSparseWithJumpBlockGreedyShrink<3> >(random_map_config.dim_,
//                                                                                  is_occupied_random,
//                                                                                  set_occupied_func,
//                                                                                  random_map.occ_grids_,
//                                                                                  random_map.occ_ids_,
//                                                                                  atoi(config.at("shrink_level").c_str()),
//                                                                                  atof(config.at("minimum_block_width").c_str()),
//                                                                                  config.at("block_path").c_str(),
//                                                                                  true);

//    auto surface_processor = std::make_shared<SurfaceProcessorSparseWithJumpBlock<3> >(oml_->getDimensionInfo(),
//                                                                                             is_occupied_func,
//                                                                                             set_occupied_func,
//                                                                                             oml_->occ_voxels_,
//                                                                                             oml_->occ_voxel_ids_,
//                                                                                             atoi(config.at("minimum_block_width").c_str()),
//                                                                                             config.at("block_path").c_str(),
//                                                                                             true);

    std::string config_file_path = config.at("config_path");
    ScenarioLoader3D scene_loader(config_file_path);
    const auto& test_cases = scene_loader.getAllTestCases();

//    auto surface_processor = std::make_shared<SurfaceProcessorSparseWithJumpBlockOctoMap>(oml_->getDimensionInfo(),
//                                                                                       is_occupied_func,
//                                                                                       set_occupied_func,
//                                                                                       oml_->occ_voxels_,
//                                                                                       oml_->occ_voxel_ids_,
//                                                                                       atoi(config.at("minimum_block_width").c_str()),
//                                                                                       config.at("block_path_oc").c_str(),
//                                                                                       true);

//    auto surface_processor = std::make_shared<SurfaceProcessor<3> >(oml_->getDimensionInfo(),
//                                                                          is_occupied_func,
//                                                                          set_occupied_func);




    std::vector<Pointi<3> > visited_pts;
    Pointis<2> neighbor = GetNeightborOffsetGrids<2>();

    gettimeofday(&tv_after, &tz);


//    freeNav::RimJump::GeneralGraphPathPlannerWithEdge<3> g2p2(tangent_graph);

#if WITH_EDGE
    GeneralGraphPathPlannerWithEdge<3> g2p2(tangent_graph);
#else
    //GeneralGraphPathPlannerWithNode<3> g2p2(tangent_graph);
#endif


    ThreadPool tp(1);

    // start (193, 72, 138) target(45, 87, 86) for complex
    // 53, 78, 56) to (52, 52, 52 simple
    // (94, 89, 126)->(160, 59, 94) complex
    // 94, 89, 126)->(160, 59, 94
    // 630, 230, 134)->(739, 278, 121 A5
    // 761, 254, 117)->(85, 90, 171 A5
    // start (44, 108, 199) target(234, 14, 20) complex los
    // start (857, 74, 25) target(85, 327, 121) A5 los
    // 38, 130, 204)->(138, 26, 0 Complex Los
    // set viewer
    viewer_3d = new Viewer3DBase();
    viewer_3d->start_[0] = 38;
    viewer_3d->start_[1] = 130;
    viewer_3d->start_[2] = 204;
    viewer_3d->target_[0] = 138;
    viewer_3d->target_[1] = 26;
    viewer_3d->target_[2] = 0;

    //std::cout << "MAX<uint_least32_t> " << MAX<uint_least32_t> << std::endl;

    viewer_thread.Schedule([&]{
        viewer_3d->init();

        pangolin::Var<bool> menu_octomap = pangolin::Var<bool>("menu.OctoMap",false, true);
        pangolin::Var<bool> menu_occupied_surface = pangolin::Var<bool>("menu.DrawOctomap",true, true);
        pangolin::Var<bool> menu_tangent_graph("menu.TangentGraph",false, true);
        pangolin::Var<bool> menu_path = pangolin::Var<bool>("menu.DrawPath",true, true);
        //pangolin::Var<bool> menu_grid("menu.GridOn",false, true);
        pangolin::Var<bool> menu_vlo = pangolin::Var<bool>("menu.DrawVoxelLine",false, true);
        pangolin::Var<bool> menu_gv = pangolin::Var<bool>("menu.DrawGreyVoxel",false, true);
        pangolin::Var<bool> menu_state = pangolin::Var<bool>("menu.DrawStart&Target",true, true);
        pangolin::Var<bool> menu_bound = pangolin::Var<bool>("menu.DrawBoundary",false, true);
        pangolin::Var<bool> menu_tangent_candidate_grid = pangolin::Var<bool>("menu.DrawTangentCandidate",false, true);
        pangolin::Var<bool> menu_set_state = pangolin::Var<bool>("menu.SetStartNow",true, true);
        pangolin::Var<bool> menu_start_plan = pangolin::Var<bool>("menu.StartPlan",false, false);
        pangolin::Var<bool> menu_rimjump = pangolin::Var<bool>("menu.RimJump*",true, true);
        pangolin::Var<bool> menu_dijk = pangolin::Var<bool>("menu.Dijkstra",true, true);
        pangolin::Var<bool> menu_astar = pangolin::Var<bool>("menu.Astar",true, true);
        pangolin::Var<bool> menu_theta = pangolin::Var<bool>("menu.Theta*",true, true);
        pangolin::Var<bool> menu_rrt = pangolin::Var<bool>("menu.RRT",true, true);
        pangolin::Var<bool> menu_jps = pangolin::Var<bool>("menu.JPS",true, true);
        pangolin::Var<bool> menu_visible_start = pangolin::Var<bool>("menu.visible_of_start",false, true);
        pangolin::Var<bool> menu_visible_target = pangolin::Var<bool>("menu.visible_of_target",false, true);
        pangolin::Var<bool> menu_surface_grids = pangolin::Var<bool>("menu.surface_grids",false, true);
        pangolin::Var<bool> menu_draw_block = pangolin::Var<bool>("menu.draw_block",false, true);
        pangolin::Var<bool> menu_draw_test_case = pangolin::Var<bool>("menu.draw_test_case",false, true);
        pangolin::Var<bool> menu_reset_view = pangolin::Var<bool>("menu.ResetView",false, false);
        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // Define Camera Render Object (for view / scene browsing)
        viewer_3d->mViewpointX = oml_->getDimensionInfo()[0]/2; // doesn't matter
        viewer_3d->mViewpointY = oml_->getDimensionInfo()[1]/2; // doesn't matter
        auto modeview_matrix = pangolin::ModelViewLookAt(oml_->getDimensionInfo()[0]/2,
                                                         oml_->getDimensionInfo()[1]/2,
                                                         3*oml_->getDimensionInfo()[2],//2*planner_3d->space_3d->max_z,
                                                         oml_->getDimensionInfo()[0]/2,
                                                         oml_->getDimensionInfo()[1]/2,
                                                         0, 0.0,-1.0, 0.0);
        // set parameters for the window
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,viewer_3d->mViewpointF,viewer_3d->mViewpointF,
                                           512,389,0.1,
                                           6000//4*oml_->getDimensionInfo()[2]
                                           ),
                modeview_matrix
        );
        MyHandler3D* handle_3d = new MyHandler3D(s_cam);
        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(handle_3d);
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        while( !pangolin::ShouldQuit() )
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if(menu_reset_view) {
                s_cam.SetModelViewMatrix(modeview_matrix);
                menu_reset_view = false;
            }
            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            if(menu_vlo) viewer_3d->show_line_for_voxel = true;
            else viewer_3d->show_line_for_voxel = false; ;
            if(menu_gv) viewer_3d->show_grey_voxel = true;
            else viewer_3d->show_grey_voxel = false;
            //if(menu_grid) DrawGrid();

            if(menu_octomap && oml_ != nullptr) {
#if OCTO_MAP
                viewer_3d->DrawOctoMapGrid(*oml_);
#else
                //viewer_3d->DrawVoxels(random_map.occ_grids_, oml_->getDimensionInfo());
                viewer_3d->DrawVoxels(surface_processor->getOccVoxels(), oml_->getDimensionInfo());
                //viewer_3d->DrawVoxels(surface_processor->getSurfacePts(), surface_processor->dimension_info_);

#endif
            }
            if(menu_occupied_surface) {
                viewer_3d->DrawVoxels(surface_processor->getSurfacePts(), surface_processor->dimension_info_);
            }
            // if(menu_surface) DrawSurface(planner_3d->space_3d);
            if(menu_state) {
//                viewer_3d->DrawPoint(viewer_3d->start_[0], viewer_3d->start_[1], viewer_3d->start_[2]);
//                viewer_3d->DrawPoint(viewer_3d->target_[0], viewer_3d->target_[1], viewer_3d->target_[2]);
//                viewer_3d->DrawLine(viewer_3d->start_[0], viewer_3d->start_[1], viewer_3d->start_[2],
//                                    viewer_3d->target_[0], viewer_3d->target_[1], viewer_3d->target_[2]);
            }
            if(menu_start_plan) {
                menu_start_plan = false;
                viewer_3d->under_planning = true;
                tp.Schedule([&]{

#if 1
                    gettimeofday(&tv_pre, &tz);
                    //visible_points = g2p2.tg_.surface_processor_->getLocalVisibleSurfaceGrids(viewer_3d->start_);
                    visible_points_start  = surface_processor->getVisibleTangentCandidates(viewer_3d->start_);
                    std::cout << " before local cross size: " << visible_points_start.size() << std::endl;
                    freeNav::GridPtrs<3> buffer;
                    for(const auto& visible_grid : visible_points_start) {
                        if(LineCrossObstacleLocal(viewer_3d->start_, visible_grid, is_occupied_func)) {
                            buffer.push_back(visible_grid);
                        }
                    }
                    std::swap(visible_points_start, buffer);
                    std::cout << " after local cross size: " << visible_points_start.size() << std::endl;
                    //visible_points_target = g2p2.tg_.surface_processor_->getVisibleTangentCandidates(viewer_3d->target_);
                    gettimeofday(&tv_after, &tz);
                    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
                    std::cout << "-- get " << visible_points_start.size() << " VisibleTangentCandidates " << build_cost << "ms" << std::endl << std::endl;
#endif

                    // line of sight check comparison
#if 1
                    // block accelerate take approximately 1/3 of raw ros check
                    int total_count = 1, visited_block;
                    double mean_time_cost_jump = 0, mean_time_cost_raw = 0;
                    for(int i=0;i<total_count;i++) {
                        gettimeofday(&tv_pre, &tz);
                        if (LineCrossObstacleWithBlockJump(viewer_3d->start_, viewer_3d->target_,
                                                           (BlockDetectorInterfacePtr<3>)surface_processor->block_detector_,
                                                           visited_pts, visited_block)) {
                            std::cout << "jump block line collide " << std::endl;
                        } else {
                            std::cout << "jump block line not collide " << std::endl;
                        }
                        gettimeofday(&tv_after, &tz);
                        build_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                        mean_time_cost_jump = mean_time_cost_jump + build_cost;
                        //std::cout << "-- block los check in " << build_cost << "ms" << std::endl;
                        gettimeofday(&tv_pre, &tz);
                        if (LineCrossObstacle(viewer_3d->start_, viewer_3d->target_, is_occupied_func, neighbor)) {
                            std::cout << "raw check line collide " << std::endl;
                        } else {
                            std::cout << "raw check line not collide " << std::endl;
                        }
                        gettimeofday(&tv_after, &tz);
                        build_cost =
                                (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                        mean_time_cost_raw = mean_time_cost_raw + build_cost;
                        //std::cout << "-- raw los check in " << build_cost << "ms" << std::endl;
                    }
                    std::cout << "-- repeat " << total_count << " times " << std::endl;
                    std::cout << "-- los check with block detect end in " << mean_time_cost_jump/total_count << "ms" << std::endl;
                    std::cout << "-- raw los check end in " << mean_time_cost_raw/total_count << "ms" << std::endl;
                    std::cout << "-- jump block visit " << visited_pts.size() << " points, " << visited_block << " blocks " << std::endl;
#endif

#if 1
                    std::cout << "-- start RimJump " << std::endl;
                    std::cout << "start " << viewer_3d->start_ << " target" << viewer_3d->target_ << std::endl;
                    plan_finish = false;
                    sg1->pt_ = viewer_3d->start_;
                    sg1->id_ = PointiToId<3>(sg1->pt_, oml_->getDimensionInfo());
                    sg2->pt_ = viewer_3d->target_;
                    sg2->id_ = PointiToId<3>(sg2->pt_, oml_->getDimensionInfo());

#if WITH_EDGE
                    ExitCode ec = g2p2.planningWithEdge(sg1->pt_, sg2->pt_,
                                                             {},
                                                             //ptcs,
                                                             etcs, ics, paths, false, true, true);
#else
//                    ExitCode ec = g2p2.planningWithNode(sg1->pt_, sg2->pt_,
//                                                        {},
//                                                        //ptcs,
//                                                        etcs, {}, paths, false, true, true);
//                    if(!paths.empty()) {
//                        cout << "-*-*- RJ path length = " << calculatePathLength(paths[0]) << endl;
//                        std::cout << " RJ Path " << paths[0] << std::endl;
//                    }
#endif

                    //ExitCode ec = g2p2.planningWithOutLength(sg1->pt_, sg2->pt_, ptcs, etcs, ics, paths, false, true, true);
//                    auto statistic_without = g2p2.getStatistic();
//                    if (ec != ExitCode::SUCCESS) {
//                        std::cout << "-- RimJump WithOut failed with " << ec << " in " << statistic_without[2] << " + " << statistic_without[3]
//                                  << "ms" << std::endl;
//                    } else {
//                        std::cout << "-- RimJump WithOut success in " << statistic_without[2] << " + " << statistic_without[3] << "ms"
//                                  << std::endl;
//                    }
#endif
                    /* try OMPL */
                    std::cout << " RRT start " << std::endl;
                    gettimeofday(&tv_pre, &tz);
                    path_rrt = {};//ompl_path_planner.plan(sg1->pt_, sg2->pt_);
                    gettimeofday(&tv_after, &tz);
                    double cost_rrt = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
                    std::cout << "-*-*- RRT path length = " << calculatePathLength(path_rrt) << ", cost " << cost_rrt << "ms" << std::endl;
                    //std::cout << " RRT Path " << path_rrt << std::endl;
                    /* end OMPL */

                    /* try JPS */
                    gettimeofday(&tv_pre, &tz);
                    std::cout << " JPS start " << std::endl;
                    path_jps  = {};//JPS_planner_ptr->plan(sg1->pt_, sg2->pt_, 1, true); // Plan from start to goal using JPS
                    gettimeofday(&tv_after, &tz);
                    double cost_jps = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
                    std::cout << "-*-*- JPS path length = " << calculatePathLength(path_jps)  << ", cost " << cost_jps << "ms" <<  std::endl;
                    //std::cout << " JPS Path " << path_jps << std::endl;
                    /* end JPS */


                    /* try 3D astar */
                    gettimeofday(&tv_pre, &tz);
                    path_astar  = {};//AstarThetaStar_3D(my_map, sg1->pt_, sg2->pt_, CN_SP_ST_ASTAR); // Plan from start to goal using JPS
                    gettimeofday(&tv_after, &tz);
                    double cost_astar = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
                    std::cout << "-*-*- 3D astar path length = " << calculatePathLength(path_astar)  << ", cost " << cost_jps << "ms" <<  std::endl;
                    std::cout << " 3D Astar Path " << path_astar << std::endl;
                    /* end 3D astar */


                    plan_finish = true;

                });
            }
            if(menu_set_state) viewer_3d->viewer_set_start = true;
            else viewer_3d->viewer_set_start = false;

            if(menu_tangent_candidate_grid) {
                for(const auto& grid_ptr : surface_processor->getTangentCandidates()) {
                    viewer_3d->DrawPoint(grid_ptr->pt_[0], grid_ptr->pt_[1], grid_ptr->pt_[2]);
                }
            }
            if(menu_bound) {
                viewer_3d->DrawBound(0, surface_processor->dimension_info_[0],
                                     0, surface_processor->dimension_info_[1],
                                     0, surface_processor->dimension_info_[2]);
                //viewer_3d->DrawVoxels(block_3d.local_minimal_pts_, cv::Vec3b(255,0,0));
                // draw block
                // following code cause un-expected random segment fault
            }
            if(menu_visible_start) {
                for(const auto& vp : visible_points_start) {
                    viewer_3d->DrawPoint(vp->pt_[0], vp->pt_[1], vp->pt_[2]);
                    viewer_3d->DrawLine(viewer_3d->start_, vp->pt_, 0, 1, 0);

                }
            }
            if(menu_surface_grids) {
                for(const auto& sfp : surface_processor->getSurfaceGrids()) {
                    viewer_3d->DrawPoint(sfp->pt_[0], sfp->pt_[1], sfp->pt_[2]);
                }
            }
            if(menu_visible_target) {
                for(const auto& vp : visible_points_target) {
                    viewer_3d->DrawPoint(vp->pt_[0], vp->pt_[1], vp->pt_[2]);
                    viewer_3d->DrawLine(viewer_3d->start_, vp->pt_, 0, 0, 1);
                }
            }
            if(menu_rimjump) {
                //viewer_3d->drawRoadMapEdges(pps);
                for(const auto& path : paths) {
                    viewer_3d->DrawPath(path);
                }
            }
            if(menu_rrt) {
                viewer_3d->DrawPath(path_rrt);
            }
            if(menu_jps) {
                viewer_3d->DrawPath(path_jps);
            }
            if(menu_astar) {
                viewer_3d->DrawPath(path_astar);
            }
            if(menu_draw_block) {
                for(int i=0; i< surface_processor->block_detector_->all_block_ptrs_.size(); i++) {
                    const auto& block_ptr =  surface_processor->block_detector_->all_block_ptrs_[i];
                    viewer_3d->DrawBlock(block_ptr->min_, block_ptr->max_,
                                         ((int)COLOR_TABLE[i%30][0])/255., ((int)COLOR_TABLE[i%30][1])/255., ((int)COLOR_TABLE[i%30][2])/255.);
                }
            }
            if(menu_draw_test_case) {
                for(const auto& test_case : test_cases) {
                    Pointi<3> pt1 = test_case.test_case_.first;
                    Pointi<3> pt2 = test_case.test_case_.second;
                    viewer_3d->DrawPoint(pt1[0], pt1[1], pt1[2]);
                    viewer_3d->DrawPoint(pt2[0], pt2[1], pt2[2]);
                }
            }
            pangolin::FinishFrame();
        }
        if(oml_) { delete oml_; }
        if(oml_shrink) { delete oml_shrink; }
    });
    return 0;
}

