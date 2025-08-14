//
// Created by yaozhuo on 8/1/25.
//

#include "../test/test_data.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/visualization/canvas/canvas.h"

#include "../algorithm/space_binary_tree/space_binary_tree_raw.h"
#include "../algorithm/space_binary_tree/space_binary_tree.h"


#include "../third_party/LazyThetaStar/pathfinding.hpp"
#include "../third_party/LazyThetaStar/JOB_adapter.h"

#include "../test/dependencies.h"

#include "../third_party/ros_motion_planner/rrt.h"
#include "../third_party/ros_motion_planner/rrt_star.h"
#include "../third_party/ros_motion_planner/rrt_connect.h"
#include "../third_party/ros_motion_planner/informed_rrt.h"
#include "../third_party/ros_motion_planner/informed_rrt.h"
#include "../third_party/ros_motion_planner/theta_star.h"
#include "../third_party/ros_motion_planner/lazy_theta_star.h"

using namespace freeNav::JOB;
using namespace freeNav;



// MapTestConfig_ost003d // ok
// MapTestConfig_orz900d // ok
// MapTestConfig_lak303d // ok
// MapTestConfig_den520d // ok
// MapTestConfig_den312d // ok
// MapTestConfig_Shanghai_0_512 // ok
// MapTestConfig_Simple_2D

// MapTestConfig_fr101 // ok
// MapTestConfig_edmonton // ok
// MapTestConfig_intel // ok, too small
// MapTestConfig_mexico // ok
// MapTestConfig_fhw_rec_001 // ok



auto map_test_config = MapTestConfig_mexico;

std::string vis_file_path    = map_test_config.at("vis_path");

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

auto is_grid_occupied1 = [](const cv::Vec3b& color) -> bool {
    if (color != cv::Vec3b::all(255)) return true;
    return false;
};

auto is_grid_occupied2 = [](const cv::Vec3b& color) -> bool {
    if (color[0] <= 240 || color[1] <= 240 || color[2] <= 240) return true;
    return false;
};

#if 1
PictureLoader loader(map_test_config.at("map_path"), is_grid_occupied2);
#else
TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
#endif


bool set_pt1 = true;
bool new_pair = false;
bool plan_finish = false;
bool draw_path = true;

//Pointi<2> pt1 = {0, 0}, pt2 = {0, 0};
Pointi<2> pt1 = {127, 272}, pt2 = {197, 397};
//Pointi<2> pt1 = {13, 3}, pt2 = {20, 12};

int path_index = 0;

int main() {

    auto dimension = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool {
        return loader.isOccupied(pt);
    };



    MSTimer mst;
    SpaceBinaryTreeAnyDimensionRaw<2> sbt_raw(is_occupied, dimension, 1);
    sbt_raw.initialize();

    std::cout << "raw SBT init in " << mst.elapsed() << "ms" << std::endl;

    mst.reset();
    SpaceBinaryTreeShrink<2> sbt(is_occupied, dimension, 4);

    std::cout << "new SBT init in " << mst.elapsed() << "ms" << std::endl;
    int count_block = 0;

    auto is_line_free_raw = [&](const Pointi<2> & p1, const Pointi<2> & p2) -> bool {
        return !sbt_raw.lineCrossObstacleRaw(p1, p2, is_occupied);
    };

    auto is_line_free_raw_sbt = [&](const Pointi<2> & p1, const Pointi<2> & p2) -> bool {
        return !sbt_raw.lineCrossObstacleSBT(p1, p2, is_occupied, count_block);
    };

    auto is_line_free_new_sbt = [&](const Pointi<2> & p1, const Pointi<2> & p2) -> bool {
        return !sbt.lineCrossObstacleSBT(p1, p2, is_occupied, count_block);
    };

    std::vector<std::pair<IS_LINE_COLLISION_FREE_FUNC<2>, std::string > >
                los_funcs = {{is_line_free_raw, "raw LOS"},
                             {is_line_free_raw_sbt, "raw SBT LOS"},
                             {is_line_free_new_sbt, "new SBT LOS"}
                            };


    unsigned char* cost_move_base_2d = new unsigned char[(dimension[0]+2)*(dimension[1]+2)];
    global_planner::getMap(cost_move_base_2d, dimension, is_occupied);


    auto theta_star = [&](const Pointi<2> & p1, const Pointi<2> & p2,
                   const IS_LINE_COLLISION_FREE_FUNC<2>& ilfr,
                   const std::string& identifier) -> Pointis<2> {
        LazyThetaStar::MyAdaptor2D my_adapter2D(dimension, is_occupied, ilfr);
        LazyThetaStar::ThetaStar pathfinder(my_adapter2D, 100.f /*weight*/);
        USTimer ust;
        auto nodePath = pathfinder.search(PointiToId<2>(p1, dimension), PointiToId<2>(p2, dimension));
        Pointis<2> retv;
        if(nodePath.empty()) {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " failed in " << ust.elapsed()/1e3 << "ms" << std::endl;
        } else {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " success in " << ust.elapsed()/1e3 << "ms" << std::endl;
            for(const auto& id : nodePath) {
                retv.push_back(IdToPointi<2>(id, dimension));
            }
        }
        return retv;
    };

    auto lazy_theta_star = [&](const Pointi<2> & p1, const Pointi<2> & p2,
                   const IS_LINE_COLLISION_FREE_FUNC<2>& ilfr,
                   const std::string& identifier) -> Pointis<2> {
        LazyThetaStar::MyAdaptor2D my_adapter2D(dimension, is_occupied, ilfr);
        LazyThetaStar::LazyThetaStar pathfinder(my_adapter2D, 100.f /*weight*/);
        USTimer ust;
        auto nodePath = pathfinder.search(PointiToId<2>(p1, dimension), PointiToId<2>(p2, dimension));
        Pointis<2> retv;
        if(nodePath.empty()) {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " failed in " << ust.elapsed()/1e3 << "ms" << std::endl;
        } else {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " success in " << ust.elapsed()/1e3 << "ms" << std::endl;
            for(const auto& id : nodePath) {
                retv.push_back(IdToPointi<2>(id, dimension));
            }
        }
        return retv;
    };

    auto pp_rrt = [&](const Pointi<2> & p1, const Pointi<2> & p2,
                      const IS_LINE_COLLISION_FREE_FUNC<2>& ilfr,
                      const std::string& identifier) -> Pointis<2> {
        USTimer ust;
        auto temp_path = global_planner::RRTRimJump(cost_move_base_2d,
                                                    dimension,
                                                    0.1,
                                                    ilfr,
                                                    pt1, pt2,
                                                    1e5,
                                                    std::max(dimension[0], dimension[1])/3);
        Pointis<2> retv;
        if(temp_path.empty()) {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " failed in " << ust.elapsed()/1e3 << "ms" << std::endl;
        } else {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " success in " << ust.elapsed()/1e3 << "ms" << std::endl;
            retv = temp_path;
        }
        return retv;
    };

    auto pp_rrt_star = [&](const Pointi<2> & p1, const Pointi<2> & p2,
                           const IS_LINE_COLLISION_FREE_FUNC<2>& ilfr,
                           const std::string& identifier) -> Pointis<2> {
        USTimer ust;
        auto temp_path = global_planner::RRTStarRimJump(cost_move_base_2d,
                                                        dimension,
                                                        0.1,
                                                        ilfr,
                                                        pt1, pt2,
                                                        1e5,
                                                        std::max(dimension[0], dimension[1])/10,
                                                        std::max(dimension[0], dimension[1])/3);
        Pointis<2> retv;
        if(temp_path.empty()) {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " failed in " << ust.elapsed()/1e3 << "ms" << std::endl;
        } else {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " success in " << ust.elapsed()/1e3 << "ms" << std::endl;
            retv = temp_path;
        }
        return retv;
    };

    auto pp_rrt_connect = [&](const Pointi<2> & p1, const Pointi<2> & p2,
                              const IS_LINE_COLLISION_FREE_FUNC<2>& ilfr,
                              const std::string& identifier) -> Pointis<2> {
        USTimer ust;
        auto temp_path = global_planner::RRTConnectRimJump(cost_move_base_2d,
                                                           dimension,
                                                           0.1,
                                                           ilfr,
                                                           pt1, pt2,
                                                           1e5,
                                                           std::max(dimension[0], dimension[1])/10);
        Pointis<2> retv;
        if(temp_path.empty()) {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " failed in " << ust.elapsed()/1e3 << "ms" << std::endl;
        } else {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " success in " << ust.elapsed()/1e3 << "ms" << std::endl;
            retv = temp_path;
        }
        return retv;
    };

    // InformedRRT not ok
    auto pp_rrt_informed = [&](const Pointi<2> & p1, const Pointi<2> & p2,
                               const IS_LINE_COLLISION_FREE_FUNC<2>& ilfr,
                               const std::string& identifier) -> Pointis<2> {
        USTimer ust;
        auto temp_path = global_planner::InformedRRTRimJump(cost_move_base_2d,
                                                           dimension,
                                                           0.1,
                                                           ilfr,
                                                           pt1, pt2,
                                                           1e5,
                                                           std::max(dimension[0], dimension[1])/10,
                                                           std::max(dimension[0], dimension[1])/3);
        Pointis<2> retv;
        if(temp_path.empty()) {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " failed in " << ust.elapsed()/1e3 << "ms" << std::endl;
        } else {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " success in " << ust.elapsed()/1e3 << "ms" << std::endl;
            retv = temp_path;
        }
        return retv;
    };

    auto pp_theta_star = [&](const Pointi<2> & p1, const Pointi<2> & p2,
                             const IS_LINE_COLLISION_FREE_FUNC<2>& ilfr,
                             const std::string& identifier) -> Pointis<2> {
        USTimer ust;
        auto temp_path = global_planner::ThetaStarRimJump(cost_move_base_2d,
                                                          dimension,
                                                          ilfr,
                                                          pt1, pt2);
        Pointis<2> retv;
        if(temp_path.empty()) {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " failed in " << ust.elapsed()/1e3 << "ms" << std::endl;
        } else {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " success in " << ust.elapsed()/1e3 << "ms" << std::endl;
            retv = temp_path;
        }
        return retv;
    };

    auto pp_lazy_theta_star = [&](const Pointi<2> & p1, const Pointi<2> & p2,
                                  const IS_LINE_COLLISION_FREE_FUNC<2>& ilfr,
                                  const std::string& identifier) -> Pointis<2> {
        USTimer ust;
        auto temp_path = global_planner::ThetaStarRimJump(cost_move_base_2d,
                                                          dimension,
                                                          ilfr,
                                                          pt1, pt2);
        Pointis<2> retv;
        if(temp_path.empty()) {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " failed in " << ust.elapsed()/1e3 << "ms" << std::endl;
        } else {
            std::cout << identifier << " plan from " << pt1 << " to " << pt2 << " success in " << ust.elapsed()/1e3 << "ms" << std::endl;
            retv = temp_path;
        }
        return retv;
    };


//    std::vector<PATH_PLANNING_FUNC<2> > path_plannings = {pp1, pp2, pp3}; // ok
//    std::vector<PATH_PLANNING_FUNC<2> > path_plannings = {pp_rrt, pp_rrt_star, pp_rrt_connect}; // ok
    std::vector<std::pair<PATH_PLANNING_FUNC_WITH_LINE<2>, std::string> >
            path_plannings = {
                              {theta_star, "ThetaStar"},
                              {lazy_theta_star, "LazyThetaStar"},
                              {pp_rrt, "RRT"},
                              {pp_rrt_star, "RRTStar"},
                              //{pp_rrt_connect, "RRTConnect"},
                              //{pp_rrt_informed, "InformedRRT"}, // not ok
//                              {pp_theta_star, "ThetaStar"}, // slow than previous one
//                              {pp_lazy_theta_star, "LazyThetaStar"} // slow than previous one
                              }; // ok

    float zoom_ratio = std::min(1800./dimension[0], 1000./dimension[1]);
    std::cout << "std::min(1800./dimension[0], 1000./dimension[1]) = " << zoom_ratio << std::endl;

    Canvas canvas("2D planner test",dimension[0], dimension[1],
                  .05, zoom_ratio);

    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = x;
                pt1[1] = y;
                set_pt1 = false;
                plan_finish = false;
                std::cout << "get point " << x << ", " << y << std::endl;
            } else {
                pt2[0] = x;
                pt2[1] = y;
                set_pt1 = true;
                std::cout << "get point " << x << ", " << y << std::endl;
                new_pair = true;
                plan_finish = false;
            }
        }
    };

    canvas.setMouseCallBack(callback);

    bool plan_path = false;
    std::vector<Pointis<2> > result_paths;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied, cv::Vec3b::all(160));

        //canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, 2, cv::Vec3b(0,255,0));
        canvas.drawCircleInt(pt1[0], pt1[1], 5, true, -1, cv::Vec3b(0,255,0));
        canvas.drawCircleInt(pt2[0], pt2[1], 5, true, -1, cv::Vec3b(255,0, 0));

        if(plan_path) {
            plan_path = false;
            std::vector<Pointis<2>> paths;
            for(int i=0; i<path_plannings.size(); i++) {
                for(int j=0; j<los_funcs.size(); j++) {
                    const auto& pp = path_plannings[i];
                    const auto& los = los_funcs[j];
                    auto retv = pp.first(pt1, pt2, los.first, pp.second + std::string(" with ") + los.second);
                    paths.push_back(retv);
                }
            }
            result_paths = paths;
        }
        if(draw_path) {
            if(!result_paths.empty() && result_paths[path_index].size() >= 2) {
                for(int i=0; i<result_paths[path_index].size()-1; i++) {
                    int x1 = result_paths[path_index][i][0];
                    int y1 = result_paths[path_index][i][1];
                    int x2 = result_paths[path_index][i+1][0];
                    int y2 = result_paths[path_index][i+1][1];
                    canvas.drawLineInt(x1, y1, x2, y2, true, 2, cv::Vec3b(0,255,0));
                }
            }
        }
        char key = canvas.show(30);
        switch (key) {
            case 32: // space
                plan_path = !plan_path;
                break;
            case 'p':
                draw_path = !draw_path;
                break;
            case 'w':
                if(!result_paths.empty()) {
                    path_index = (path_index + 1) % result_paths.size();
                    std::cout << "path index = " << path_index << std::endl;
                }
                break;
            case 's':
                if(!result_paths.empty()) {
                    path_index = (path_index + result_paths.size() - 1) % result_paths.size();
                    std::cout << "path index = " << path_index << std::endl;
                }
                break;
            default:
                break;
        }
    }

    delete [] cost_move_base_2d;

}