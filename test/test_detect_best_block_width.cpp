//
// Created by yaozhuo on 2023/5/18.
//
#include "gtest/gtest.h"

#include "../freeNav-base/test/test_data.h"

#include "octomap/octomap.h"

#include "../algorithm/block_detect.h"
#include "../algorithm/line_of_sight_jump_between_block.h"
#include "../algorithm/surface_process_jump_block.h"
#include "../freeNav-base/dependencies/3d_textmap/voxel_loader.h"

using namespace freeNav;
using namespace freeNav::JOB;

// MapTestConfig_Full4 // run out of space, shrink space 4
// MapTestConfig_Simple // success
// MapTestConfig_DA2
// MapTestConfig_DC1 // one bywave, need 8~14s
// MapTestConfig_Complex
// MapTestConfig_A1
// MapTestConfig_FA2
// MapTestConfig_A5
auto config = MapTestConfig_Complex;

std::string file_path = "/home/yaozhuo/code/free-nav/resource/binary/fr_campus.vis";

TextMapLoader_3D* oml_ = nullptr;

double getTotalTimeCostOfMapWithWidth(DimensionLength* dimension_info,
                                      IS_OCCUPIED_FUNC<3> is_occupied,
                                      SET_OCCUPIED_FUNC<3> set_occupied,
                                      const std::vector<Pointi<3> >& occ_grids,
                                      const IdSet& occ_voxel_ids,
                                      int max_test_count,
                                      int block_width,
                                      int max_random_select = 100,
                                      int repeat_times = 100) {

    // random experiment data
    Id total_index = getTotalIndexOfSpace<3>(dimension_info);
    Id temp_id;

    Pointis<3> pts;
    Pointi<3> pt;
    struct timezone tz;
    struct timeval tv_pre, tv_pre_for_count;
    struct timeval tv_after;

    //auto bd = std::make_shared<BlockDetector<3> >(dimension_info, is_occupied, corner_grids, bw, "", true);
    auto surface_processor =
            std::make_shared<SurfaceProcessorSparseWithJumpBlock<3> >(dimension_info, is_occupied, set_occupied,
                                                                      occ_grids, occ_voxel_ids,
                                                                      block_width,
                                                                      "",
                                                                      false);
    double total_time_cost = 0;
    for(int i=0; i<max_test_count; i++ ) {
        // 1, pick random start / target
        pts.clear();
        for(int j=0; j < max_random_select; j++) {
            temp_id = (Id)(total_index*rand() / double(RAND_MAX));
            //std::cout << "temp_id " << temp_id << std::endl;
            pt = IdToPointi<3>(temp_id, dimension_info);
            if(is_occupied(pt)) { continue; }
            else {
                pts.push_back(pt);
                if(pts.size() == 2) {
                    if(pts[0] == pts[1]) {
                        pts.pop_back();
                        continue;
                    }
                    break;
                }
            }
        }
        if(pts.size() != 2) {
            //std::cout << " index " << i << " find no free start/target pair" << std::endl;
            continue;
        }
        // 2, do los check and accumulate total time cost
        Pointis<3> count_of_visited_pt; int count_of_visited_block;
        gettimeofday(&tv_pre, &tz);
        for(int j=0; j<repeat_times; j++) {
            //LineCrossObstacleWithBlockJump(pts[0], pts[1], (BlockDetectorInterfacePtr<3>)bd, count_of_visited_pt, count_of_visited_block);
            surface_processor->lineCrossObstacleWithVisitedPoint(pts[0], pts[1], count_of_visited_pt, count_of_visited_block);
        }
        gettimeofday(&tv_after, &tz);
        double time_interval = (tv_after.tv_sec-tv_pre_for_count.tv_sec)+(tv_after.tv_usec-tv_pre_for_count.tv_usec)/10e6;
        if(time_interval > .1) {
            std::cout << "--finish :  " << i << " cases " << std::endl;
            gettimeofday (&tv_pre_for_count , &tz);
        }
        double los_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        total_time_cost += los_cost;
    }
    std::cout << "block width = " << block_width << ", mean time cost " << total_time_cost/max_test_count << " in " << max_test_count << " test " << std::endl;
    return total_time_cost;
}


// traversal all block width to detect the optimal block width
int detectMinimumTimeCostBlockWidth(DimensionLength* dimension_info,
                                    IS_OCCUPIED_FUNC<3> is_occupied,
                                    SET_OCCUPIED_FUNC<3> set_occupied,
                                    const std::vector<Pointi<3> >& occ_grids,
                                    const IdSet& occ_voxel_ids,
                                    int max_test_count,
                                    int max_block_width = 100,
                                    int resolution = 5,
                                    int max_random_select = 100,
                                    int repeat_times = 100
) {
    double minimum_time_cost = MAX<double>;
    int best_width;
    double total_time_cost;
    std::vector<std::pair<int, double> > block_width_and_time_cost;
    for(int bw = resolution; bw <= max_block_width; bw +=resolution ) {
        total_time_cost = getTotalTimeCostOfMapWithWidth(dimension_info,
                                                         is_occupied, set_occupied,
                                                         occ_grids, occ_voxel_ids,
                                                         max_test_count, bw,
                                                         max_random_select,
                                                         repeat_times);
        block_width_and_time_cost.push_back(std::make_pair(bw, total_time_cost/max_test_count));
        if(total_time_cost < minimum_time_cost) {
            minimum_time_cost = total_time_cost;
            best_width = bw;
        }
    }
    for(const auto& bw_and_time : block_width_and_time_cost) {
        std::cout << " block width = " << bw_and_time.first << ", mean time cost " << bw_and_time.second << " in " << max_test_count << " test " << std::endl;
    }
    return best_width;
}

int bisectionDetectMinimumTimeCostBlockWidth(DimensionLength* dimension_info,
                                            IS_OCCUPIED_FUNC<3> is_occupied,
                                            SET_OCCUPIED_FUNC<3> set_occupied,
                                            const std::vector<Pointi<3> >& occ_grids,
                                            const IdSet& occ_voxel_ids,
                                            int max_test_count,
                                            int min_block_width = 5,
                                            int max_block_width = 100,
                                            int max_random_select = 100,
                                            int repeat_times = 100
) {
    int left_block_width = min_block_width, right_block_width = max_block_width;

    double left_time_cost = getTotalTimeCostOfMapWithWidth(dimension_info,
                                                           is_occupied, set_occupied,
                                                           occ_grids, occ_voxel_ids,
                                                           max_test_count, min_block_width,
                                                           max_random_select,
                                                           repeat_times);

    double right_time_cost = getTotalTimeCostOfMapWithWidth(dimension_info,
                                                           is_occupied, set_occupied,
                                                           occ_grids, occ_voxel_ids,
                                                           max_test_count, max_block_width,
                                                           max_random_select,
                                                           repeat_times);
    int count = 0;
    while(right_block_width - left_block_width > 2) {
        int medium_left_block_width  = left_block_width + (right_block_width - left_block_width) * 1 / 3;
        int medium_right_block_width = left_block_width + (right_block_width - left_block_width) * 2 / 3;

        double medium_left_time_cost = getTotalTimeCostOfMapWithWidth(dimension_info,
                                                                 is_occupied, set_occupied,
                                                                 occ_grids, occ_voxel_ids,
                                                                 max_test_count, medium_left_block_width,
                                                                 max_random_select,
                                                                 repeat_times);

        double medium_right_time_cost = getTotalTimeCostOfMapWithWidth(dimension_info,
                                                                      is_occupied, set_occupied,
                                                                      occ_grids, occ_voxel_ids,
                                                                      max_test_count, medium_right_block_width,
                                                                      max_random_select,
                                                                      repeat_times);

//        if( medium_left_time_cost < medium_right_time_cost) {
//
//        }

        std::cout << count << "th iteration: block width ";
        std::cout << left_block_width << " " << medium_left_block_width << " " << medium_right_block_width << " " << right_block_width << " / time cost ";
        std::cout << left_time_cost << " " << medium_left_time_cost << " " << medium_right_time_cost << " " << right_time_cost << std::endl;

        if(left_time_cost        > medium_left_time_cost &&
          medium_right_time_cost > right_time_cost &&
          medium_left_time_cost >  medium_right_time_cost) {
            // left is increasing, stop
            //std::cout << " time cost continue decreasing " << std::endl;
            //return right_block_width;
            left_time_cost = medium_left_time_cost;
            left_block_width = medium_left_block_width;
        }
        if(left_time_cost          < medium_left_time_cost &&
           medium_right_time_cost  < right_time_cost &&
           medium_left_time_cost  < medium_right_time_cost) {
            // left is increasing, stop
            //std::cout << " time cost continue increasing " << std::endl;
            //return left_block_width;
            right_time_cost = medium_right_time_cost;
            right_block_width = medium_right_block_width;
        }
        // when left and right is the smallest
        if(left_time_cost < medium_left_time_cost &&
           left_time_cost < medium_right_time_cost &&
           left_time_cost < right_time_cost) {
            right_time_cost = medium_right_time_cost;
            right_block_width = medium_right_block_width;
        } else if(right_time_cost < medium_left_time_cost &&
                  right_time_cost < medium_right_time_cost &&
                  right_time_cost < left_time_cost) {
            left_time_cost = medium_left_time_cost;
            left_block_width = medium_left_block_width;
        } else {
            // when medium left and medium right is the smallest
            if (medium_left_time_cost < medium_right_time_cost) {
                right_time_cost = medium_right_time_cost;
                right_block_width = medium_right_block_width;
            } else {
                left_time_cost = medium_left_time_cost;
                left_block_width = medium_left_block_width;
            }
        }
        //std::cout << "after block width " << std::endl;
        //std::cout << left_block_width << " " << right_block_width << std::endl;
        count ++;
    }
    std::cout << "after " << count << " iteration, find best block width "
              << (left_block_width + right_block_width) / 2 << std::endl;
    return (left_block_width + right_block_width) / 2;
}

void detectBestBlockWidth(const SingleMapTestConfig<3>& current_config) {
#if OCTO_MAP
    oml_ = new OctoMapLoader("/home/yaozhuo/code/free-nav/resource/map/fr_campus.bt", 6);
#else
    std::string map_path = current_config.at("map_path");
    std::cout << " map path: " << map_path << std::endl;
    oml_ = new TextMapLoader_3D(map_path, atoi(current_config.at("shrink_level").c_str())); // shrink to enable load
    file_path = current_config.at("vis_path");
#endif
    // load octomap
    auto is_occupied = [](const Pointi<3> & pt) -> bool { return oml_->isOccupied(pt); };
    auto set_occupied = [](const Pointi<3> & pt) { oml_->setOccupied(pt); };

    IS_OCCUPIED_FUNC<3> is_occupied_func = is_occupied;

    SET_OCCUPIED_FUNC<3> set_occupied_func = set_occupied;


//    int best_block_width = detectMinimumTimeCostBlockWidth(oml_->getDimensionInfo(),
//                                                           is_occupied,
//                                                           set_occupied,
//                                                           oml_->occ_voxels_,
//                                                           oml_->occ_voxel_ids_,
//                                                           10000,
//                                                           100,
//                                                           5,
//                                                           100,
//                                                           1
//    );

    int best_block_width = bisectionDetectMinimumTimeCostBlockWidth(oml_->getDimensionInfo(),
                                                                   is_occupied,
                                                                   set_occupied,
                                                                   oml_->occ_voxels_,
                                                                   oml_->occ_voxel_ids_,
                                                                   10000,
                                                                   5, // jump over block is more time consuming than check a grid state, so block shouldn't be too small
                                                                   getMinimumDimension<3>(oml_->getDimensionInfo())/2, // maximum block width is determine by the size of the space
                                                                   100,
                                                                   100
    );

    std::cout << "best block width " << best_block_width << std::endl;
}

SingleMapTestConfigs<3> configs = {
            MapTestConfig_Simple,
            MapTestConfig_Complex, // AC
            MapTestConfig_A1, // AC
            MapTestConfig_A2, // AC
            MapTestConfig_A3, // AC
            MapTestConfig_A4, // AC
            MapTestConfig_A5, // AC

            MapTestConfig_BC1, //
            MapTestConfig_BC2, // AC
            MapTestConfig_DA1, // AC
            MapTestConfig_DA2, // AC
            MapTestConfig_DB1, // AC
            MapTestConfig_DB2, // AC

            MapTestConfig_DC1, // AC
            MapTestConfig_DC2, // AC
            MapTestConfig_EB1, // AC
            MapTestConfig_EB2, // AC
            MapTestConfig_EC1, // AC
            MapTestConfig_EC2, // AC
//            MapTestConfig_Full4
};

int main() {
    for(const auto& config : configs) {
        detectBestBlockWidth(config);
    }
}