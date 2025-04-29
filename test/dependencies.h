//
// Created by yaozhuo on 2025/4/29.
//

#ifndef JUMPOVERBLOCK_DEPENDENCIES_H
#define JUMPOVERBLOCK_DEPENDENCIES_H

#include "../algorithm/space_binary_tree/space_binary_tree.h"
#include "dynamic_obstacles.h"

namespace freeNav::JOB {

    template<Dimension N>
    void SpaceBinaryTreeVarify(DimensionLength* temp_dim,
                    const IS_OCCUPIED_FUNC<N>& isoc_temp,
                    const SpaceBinaryTree<N>& sbt) {

        Id total_index = getTotalIndexOfSpace<N>(temp_dim);
        // debug: check whether SBT have the same state as map
        //        and each block is passable
        for(Id id=0; id<total_index; id++) {
            Pointi<N> pt = IdToPointi<N>(id, temp_dim);
            assert(isoc_temp(pt) == sbt.isOccupied(pt));
            if(sbt.block_ptr_map_[id] != nullptr) {
                assert(isoc_temp(pt) == false);
            }
        }

    }

    // times_of_test do how many times of LOS compare
    // each compare use how many times of sample to find a point
    template<Dimension N>
    void LOSCompare(DimensionLength* temp_dim,
                    const IS_OCCUPIED_FUNC<N>& isoc_temp,
                    const SpaceBinaryTree<N>& sbt,
                    int times_of_test = 1e6,
                    int max_sample_times = 1e3) {

        SpaceBinaryTreeVarify<N>(temp_dim, isoc_temp, sbt);

        struct timezone tz;
        struct timeval tv_pre;
        struct timeval tv_after;

        Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();

        Id total_index = getTotalIndexOfSpace<N>(temp_dim);

        double sum_1 = 0, sum_2 = 0;
        int success_count = 0;
        for(int i=0; i<times_of_test; i++) {
            // random pick two passable point
            Id id1 = 0, id2 = 0;
            Pointi<N> pt1, pt2;
            int count = max_sample_times;
            while(count >= 0) {
                id1 = rand() % total_index;
                pt1 = IdToPointi<N>(id1, temp_dim);
                if (!isoc_temp(pt1)) {
                    break;
                } else {
                    count --;
                }
            }
            if (isoc_temp(pt1)) {
                continue;
            }
            count = max_sample_times;
            while(count >= 0) {
                id2 = rand() % total_index;
                pt2 = IdToPointi<N>(id2, temp_dim);
                if (!isoc_temp(pt2)) {
                    break;
                } else {
                    count --;
                }
            }
            if (isoc_temp(pt2)) {
                continue;
            }
            //std::cout << "do LOS between " << pt1 << ", " << pt2 <<  std::endl;
            gettimeofday(&tv_pre, &tz);
            bool isoc1 = LineCrossObstacle<N>(pt1, pt2, isoc_temp, neighbor);
            gettimeofday(&tv_after, &tz);
            double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            sum_1 = sum_1 + time_cost1;
            Pointis<N> visited_pt;
            int count_of_block;
            gettimeofday(&tv_pre, &tz);
            bool isoc2 = sbt.lineCrossObstacle(pt1, pt2, visited_pt, count_of_block);
            gettimeofday(&tv_after, &tz);
            double time_cost2 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            sum_2 = sum_2 + time_cost2;
            success_count ++;
            if(isoc1 != isoc2) {
                std::cout << i << " th test failed, pt1/pt2 = " << pt1 << " / " << pt2 << std::endl;
                // assert classic LOS check and SBT's LOS check have the same result
                assert(isoc1 == isoc2);
            }
        }

        std::cout << success_count <<  " LOS test, mean raw LOS time cost = " << sum_1/(double)success_count
                  << ", mean SBT LOS time cost = " << sum_2/(double)success_count << std::endl;

    }

    // half obstacle is circle and another half is block obstacle
    template<Dimension N>
    void massiveSBTLOSCompareTest(int random_times, // how many times of randomize for a map
                                  int repeat_times, // how many times of LOS for a randomize
                                  const std::vector<int>& width_of_space,
                                  const std::vector<int>& number_of_obstacles,
                                  int min_radius = 5,
                                  int max_radius = 10,
                                  int min_block_width = 10,
                                  int max_block_width = 20) {


        for(const auto& width : width_of_space) {
            for(const auto& count : number_of_obstacles) {
                ObstaclePtrs<N> obs;

                CircleObstaclePtrs<N> co = generateRandomCircleObstacles<N>(count/2, min_radius, max_radius);
                obs.insert(obs.end(), co.begin(), co.end());

                Pointi<N> min_pt, max_pt;
                min_pt.setAll(min_block_width);
                max_pt.setAll(max_block_width);
                BlockObstaclePtrs<N> bo = generateRandomBlockObstacles<N>(count/2, min_pt, max_pt);
                obs.insert(obs.end(), co.begin(), co.end());


                DimensionLength dim[N];
                for(int d=0; d<N; d++) {
                    dim[d] = width;
                }
                auto is_occupied = [&](const Pointi<N> & pt) -> bool {
                    if(isOutOfBoundary(pt, dim)) {
                        return true;
                    }
                    return false;
                };
                SpaceBinaryTree<N> sbt(is_occupied, dim);

                DynamicObstacles<N> dynamic_obstacles(dim, obs);

                Id total_index = getTotalIndexOfSpace<N>(dim);
                std::vector<bool> temp_map(total_index, false);
                std::cout << N << " dimension space, width = " << width << ", number of obstacles = " << count << std::endl;

                for(int i=0; i<random_times; i++) {

                    dynamic_obstacles.random();

                    for(const auto& pre_pt : dynamic_obstacles.getPreviousOccupationPoints()) {
                        sbt.setOccupiedState(pre_pt, false);
                        temp_map[PointiToId(pre_pt, dim)] = false;
                    }
                    for(const auto& cur_pt : dynamic_obstacles.getCurrentOccupationPoints()) {
                        sbt.setOccupiedState(cur_pt, true);
                        temp_map[PointiToId(cur_pt, dim)] = true;
                    }
                    // construct local update isoc
                    auto is_occupied_temp = [&](const Pointi<N> & pt) -> bool {
                        if(isOutOfBoundary(pt, dim)) {
                            return true;
                        }
                        return temp_map[PointiToId(pt, dim)];
                    };

                    LOSCompare<N>(dim, is_occupied_temp, sbt);

                }

            }
        }
    }


}

#endif //JUMPOVERBLOCK_DEPENDENCIES_H
