//
// Created by yaozhuo on 2023/4/17.
//

#include "rim_jump/los_check_for_sparse/map_down_sampler.h"
#include "rim_jump/los_check_for_sparse/block_detect.h"
#include "rim_jump/los_check_for_sparse/block_detector_greedy.h"

#include "2d_grid/text_map_loader.h"

#include "gtest/gtest.h"
#include "canvas/canvas.h"
#include "dependencies/test_data.h"
#include "rim_jump/los_check_for_sparse/line_of_sight_jump_between_block.h"

//#include <CGAL/Simple_cartesian.h>
//#include <CGAL/Quadtree.h>
//#include <CGAL/Random.h>
#include "octomap/octomap.h"
#include "dependencies/random_map_generator.h"

#include "rim_jump/basic_elements/distance_map_update.h"

// https://github.com/PathPlanning/3D-AStar-ThetaStar

// Type Declarations
//typedef CGAL::Simple_cartesian<int> Kernel;
//typedef Kernel::Point_2 Point_2;
//typedef std::vector<Point_2> Point_vector;
//typedef CGAL::Quadtree<Kernel, Point_vector> Quadtree;

using namespace freeNav::RimJump;
using namespace freeNav;

auto is_grid_occupied1 = [](const cv::Vec3b& color) -> bool {
    if (color != cv::Vec3b::all(255)) return true;
    return false;
};

auto is_grid_occupied2 = [](const cv::Vec3b& color) -> bool {
    if (color[0] <= 200 || color[1] <= 200 || color[2] <= 200) return true;
    return false;
};

auto is_char_occupied = [](const char& value) -> bool {
    if (value != '.' && value != 'G' && value != 'S') return true;
    return false;
};

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

#define USE_PIC_MAP 0
#if USE_PIC_MAP
// tarjan search raise hardfault in map fr-campus-rectified
std::string file_name = "map_001.png";
PictureLoader loader("/home/yaozhuo/code/free-nav/resource/map/" + file_name, is_grid_occupied2);
std::string vis_file_path = "/home/yaozhuo/code/free-nav/resource/binary/" + file_name + ".vis";

// MapTestConfig_Entanglement;
// MapTestConfig_FloodedPlains;
// MapTestConfig_dustwallowkeys;
// MapTestConfig_Boston_0_1024;
// MapTestConfig_TheFrozenSea;
// MapTestConfig_maze512_4_8
// MapTestConfig_8room_002
// MapTestConfig_Aurora
// MapTestConfig_Berlin_1_256
int zoom_rate = 1;

#else

//MapTestConfig_TheFrozenSea
//MapTestConfig_FloodedPlains
//MapTestConfig_maze512_4_8
//MapTestConfig_Berlin_1_256
//MapTestConfig_maze512_4_0
//MapTestConfig_Entanglement
//MapTestConfig_Aurora
//MapTestConfig_dustwallowkeys // with parse error
//MapTestConfig_8room_002
//MapTestConfig_Boston_0_1024
//MapTestConfig_random512_10_0 // RJ success
//MapTestConfig_random512_35_2
//MapTestConfig_Denver_2_512
// MapTestConfig_Milan_1_512
//MapTestConfig_Sydney_1_256
// MapTestConfig_Shanghai_0_512
auto map_test_config = MapTestConfig_Shanghai_0_512;
std::string vis_file_path    = map_test_config.at("vis_path");

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
int zoom_rate = 1;

#endif

auto dimension = loader.getDimensionInfo();

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const Pointi<2> & pt) { loader.setOccupied(pt); };

IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;

bool set_pt1 = true;
bool new_pair = false;
bool plan_finish = false;

GridPtr<2> sg1 = std::make_shared<freeNav::RimJump::Grid<2>>(),
                             sg2 = std::make_shared<freeNav::RimJump::Grid<2>>();

Pointi<2> pt1, pt2;

int total_sample_level = 4; // including the raw map
int current_show_level = 0;

MapDownSampler<2> down_sampled_map(is_occupied, dimension, total_sample_level - 1);

auto is_occupied_downsample = [](const Pointi<2> & pt) -> bool {
    return down_sampled_map.isOccupiedMiniMap(pt);
};

IS_OCCUPIED_FUNC<2> is_occupied_downsample_func = is_occupied_downsample;

ThreadPool tp;


struct timezone tz;
struct timeval  tv_pre;
struct timeval  tv_after;

TEST(MAP_DOWN_SAMPLER, down_sampler_2d) {

    Canvas canvas("RimJump::down_sampler_2d", dimension[0], dimension[1], .05, zoom_rate);

    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        //canvas.drawGridMap(dimension, is_occupied_func);
        canvas.drawGridMap(down_sampled_map, current_show_level);

        //canvas.drawGridMap(down_sampled_map.dimension_infos_.back(), is_occupied_downsample_func);

        char key = canvas.show(300);
        switch (key) {
            case 'w':
                current_show_level ++;
                current_show_level = current_show_level % total_sample_level;
                break;
            case 's':
                current_show_level --;
                current_show_level = (current_show_level + total_sample_level) % total_sample_level;
                break;
            default:
                break;
        }
        //break;
    }
}

TEST(BlockDetector, initAllDirectionLocalMoves) {
    std::vector<Pointis<3> > all_direction_local_move_in_3d = initAllDirectionLocalMoves<3>();
    for(const auto& each_direction : all_direction_local_move_in_3d) {
        std::cout << each_direction << std::endl;
    }
}


// TODO: considering detect block via machine learning ?
TEST(BlockDetector, BlockDetectorFull) {

    //MapDownSampler<2> down_sampled_map(is_occupied, dimension, total_sample_level - 1);


    //zoom_rate = min(2560/dimension[0], 1280/dimension[1]);
//    Canvas canvas("RimJump::dist_map",
//                  down_sampled_map.dimension_infos_.back()[0],
//                  down_sampled_map.dimension_infos_.back()[1], .05, zoom_rate);
//
//    BlockDetector<2> block_detect(is_occupied_downsample, down_sampled_map.dimension_infos_.back());

    Canvas canvas("RimJump::BlockDetector",
                  dimension[0],
                  dimension[1], .05, zoom_rate);

    auto surface_processor = std::make_shared<SurfaceProcessor<2> >(dimension, is_occupied_func, set_occupied_func);

    surface_processor->surfaceGridsDetection();

    gettimeofday(&tv_pre, &tz);

    BlockDetectorGreedyPtr<2> block_detect = std::make_shared<BlockDetectorGreedy<2> >(
            dimension, is_occupied_func, surface_processor->getSurfacePts(), 5);

//    BlockDetectorPtr<2> block_detect = std::make_shared<BlockDetector<2> >(
//            dimension, is_occupied_func, surface_processor->getSurfaceGrids(), 40);

    gettimeofday(&tv_after, &tz);

    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- block detect end in " << build_cost << "ms" << std::endl;

    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == CV_EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = x;
                pt1[1] = y;
                sg1->pt_ = pt1;
                sg1->id_ = PointiToId<2>(pt1, dimension);
                set_pt1 = false;
                plan_finish = false;
                std::cout << "get point " << x << ", " << y << std::endl;
            } else {
                pt2[0] = x;
                pt2[1] = y;
                sg2->pt_ = pt2;
                sg2->id_ = PointiToId<2>(pt2, dimension);
                set_pt1 = true;
                std::cout << "get point " << x << ", " << y << std::endl;
                new_pair = true;
                plan_finish = false;
            }
        }
    };

    canvas.setMouseCallBack(callback);


    bool draw_dist_map = false,
         draw_node = false,
         draw_local_minimal = false,
         draw_block = false,
         draw_iso = false,
         draw_block_ptr = false,
         draw_dist_map_updated = false;
    Pointis<1> neighbor = GetNeightborOffsetGrids<1>();
    std::vector<Pointi<2> > visited_pts;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        //canvas.drawGridMap(down_sampled_map.dimension_infos_.back(), is_occupied_downsample_func);
        canvas.drawGridMap(dimension, is_occupied);
        if(new_pair) {
            new_pair = false;
            if (tp.pool_[0].joinable() && !plan_finish)
                tp.Schedule([&] {
                    double mean_time_cost_jump = 0, mean_time_cost_raw = 0;
                    int total_count = 1;
                    for(int i=0; i<total_count; i++) {
                        gettimeofday(&tv_pre, &tz);
//                        if (LineCrossObstacleWithBlockJump(pt1, pt2,
//                                                           block_detect,
//                                                           visited_pts)) {
//                            std::cout << "jump block line collide " << std::endl;
//                        } else {
//                            std::cout << "jump block line not collide " << std::endl;
//                        }
                        gettimeofday(&tv_after, &tz);
                        build_cost =
                                (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                        mean_time_cost_jump = mean_time_cost_jump + build_cost;
                        gettimeofday(&tv_pre, &tz);
                        if (LineCrossObstacle(pt1, pt2, is_occupied_func, neighbor)) {
                            std::cout << "raw check line collide " << std::endl;
                        } else {
                            std::cout << "raw check line not collide " << std::endl;
                        }
                        gettimeofday(&tv_after, &tz);
                        build_cost =
                                (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                        mean_time_cost_raw = mean_time_cost_raw + build_cost;
                    }
                    std::cout << "-- repeat " << total_count << " times " << std::endl;
                    std::cout << "-- los check with block detect end in " << mean_time_cost_jump/total_count << "ms" << std::endl;
                    std::cout << "-- raw los check end in " << mean_time_cost_raw/total_count << "ms" << std::endl;
                    std::cout << "-- jump block visit " << visited_pts.size() << " points " << std::endl;
                });
        }
        if(draw_node) {
            canvas.drawPointiCircles(block_detect->surface_nodes_, cv::Vec3b(0,255,0), 5, -1);
        }
        if(draw_local_minimal) {
            canvas.drawPointiCircles(block_detect->local_maximal_pts_, cv::Vec3b(255,0,0), 7, -1);
        }
        if(draw_iso) {
            //canvas.drawGrids(block_detect->isolated_minimal_pts_, cv::Vec3b(255,0,255));
            canvas.drawPointiCircles(block_detect->isolated_minimal_pts_, cv::Vec3b(255,0,255), 7, -1);
        }
        if(draw_dist_map) {
            canvas.draw_DistMap(block_detect->dimension_info_, block_detect->dist_map_);
        }
        if(draw_dist_map_updated) {
            canvas.draw_DistMap(block_detect->dimension_info_, block_detect->dist_map_updated_);
        }
        if(draw_block_ptr) {
            //canvas.draw_DistMap(block_detect.dimension_info_, block_detect.dist_map_);
            int total_count = getTotalIndexOfSpace<2>(dimension);
            for(int i=0; i<total_count; i++) {
                if(block_detect->block_ptr_map_[i] != nullptr) {
                    Pointi<2> pt = IdToPointi<2>(i, dimension);
                    Pointi<2> center_pt = block_detect->block_ptr_map_[i]->max_;
                    Id center_id = PointiToId(center_pt, dimension);
                    canvas.drawGrid(pt[0], pt[1], COLOR_TABLE[center_id%30]);
                }
            }
        }
        if(draw_block) {
            for(const auto& block_ptr : block_detect->all_block_ptrs_) {
                canvas.draw_Block(block_ptr->min_, block_ptr->max_);
            }
        }
        canvas.drawGrids(visited_pts);
        canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, 2, cv::Vec3b(0,255,0));
        //canvas.drawCircleInt(pt1[0], pt1[1], 5, true, -1, COLOR_TABLE[0]);
        //canvas.drawCircleInt(pt2[0], pt2[1], 5, true, -1, COLOR_TABLE[1]);
        char key = canvas.show(300);
        switch (key) {
            case 32:
                draw_dist_map = !draw_dist_map;
                break;
            case 't':
                draw_node = !draw_node;
                break;
            case 'l':
                draw_local_minimal = !draw_local_minimal;
                break;
            case 'b':
                draw_block = !draw_block;
                break;
            case 'i':
                draw_iso = !draw_iso;
                break;
            case 'p':
                draw_block_ptr = !draw_block_ptr;
                break;
            case 'u':
                draw_dist_map_updated = !draw_dist_map_updated;
                break;
            default:
                break;
        }
    }
}

//TEST(CGAL, example)
//{
//    std::cout << "CGAL test" << std::endl;
//    CGAL::Random r;
//    Point_vector points_2d;
//    for (std::size_t i = 0; i < 5; ++ i)
//        points_2d.emplace_back(r.get_int(-1., 1.),
//                               r.get_int(-1., 1.));
//    Quadtree quadtree(points_2d);
//    quadtree.refine(10, 5);
//}

TEST(RandomSampleMap, RandomSampleMap2D) {

    //zoom_rate = std::min(1, min(2560/dimension[0], 1280/dimension[1]));
    zoom_rate = 1;
    auto current_map_config = grid_2D_8;

    Canvas canvas("RandomSampleMap::RandomSampleMap2D",
                  current_map_config.dim_[0],
                  current_map_config.dim_[1], .05, zoom_rate);
    gettimeofday(&tv_pre, &tz);
    RandomMapGenerator<2> random_map(current_map_config.dim_,
                                     current_map_config.cubic_half_width_,
                                     current_map_config.cubic_number_,
                                     current_map_config.random_file_path_, false);
    gettimeofday(&tv_after, &tz);

    auto is_occupied_random = [&](const Pointi<2> & pt) -> bool {
        if(isOutOfBoundary(pt, current_map_config.dim_)) { return true; }
        Id id = PointiToId(pt, current_map_config.dim_);
        //return random_map.grid_map_[id];
        return random_map.occ_ids_.find(id) != random_map.occ_ids_.end();
    };


    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- random sample end in " << build_cost << "ms" << std::endl;

    bool draw_raw_map = false,
         draw_random_map = false;
    std::vector<Pointi<2> > visited_pts;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        //canvas.drawGridMap(down_sampled_map.dimension_infos_.back(), is_occupied_downsample_func);
        if(draw_random_map) {
            canvas.drawGridMap(current_map_config.dim_, is_occupied_random);
        }
        char key = canvas.show(300);
        switch (key) {
            case 32:
                draw_raw_map = !draw_raw_map;
                break;
            case 's':
                draw_random_map = !draw_random_map;
                break;
            default:
                break;
        }
    }
}

TEST(DistanceMapUpdate, test) {

    zoom_rate = min(2560/dimension[0], 1280/dimension[1]);

    Canvas canvas("RimJump::BlockDetector",
                  dimension[0],
                  dimension[1], .05, zoom_rate);

    gettimeofday(&tv_pre, &tz);

    DistanceMapUpdaterPtr<2> distance_map = std::make_shared<DistanceMapUpdater<2>>(is_occupied_func, dimension);

    gettimeofday(&tv_after, &tz);

    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- block detect end in " << build_cost << "ms" << std::endl;

    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == CV_EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = x;
                pt1[1] = y;
                sg1->pt_ = pt1;
                sg1->id_ = PointiToId<2>(pt1, dimension);
                set_pt1 = false;
                plan_finish = false;
                std::cout << "get point " << x << ", " << y << std::endl;
            } else {
                pt2[0] = x;
                pt2[1] = y;
                sg2->pt_ = pt2;
                sg2->id_ = PointiToId<2>(pt2, dimension);
                set_pt1 = true;
                std::cout << "get point " << x << ", " << y << std::endl;
                new_pair = true;
                plan_finish = false;
            }
        }
    };

    canvas.setMouseCallBack(callback);


    bool draw_dist_map = false;
    Pointis<1> neighbor = GetNeightborOffsetGrids<1>();
    std::vector<Pointi<2> > visited_pts;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied);

        if(draw_dist_map) {
            canvas.draw_DistMap(distance_map->dimension_info_,
                                distance_map->dist_map_,
                                Pointi<2>(),
                                distance_map->max_dist_,
                                distance_map->min_dist_);
        }

        char key = canvas.show(300);
        switch (key) {
            case 32:
                draw_dist_map = !draw_dist_map;
                break;
            default:
                break;
        }
    }
}