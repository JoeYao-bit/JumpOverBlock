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
                               const SpaceBinaryTreePtr<N>& sbt) {

        struct timezone tz;
        struct timeval tv_pre;
        struct timeval tv_after;
        gettimeofday(&tv_pre, &tz);

        std::cout << "start SpaceBinaryTree varify thread " << std::endl;

        Id total_index = getTotalIndexOfSpace<N>(temp_dim);
        // debug: check whether SBT have the same state as map
        //        and each block is passable
        for(Id id=0; id<total_index; id++) {
            Pointi<N> pt = IdToPointi<N>(id, temp_dim);
            assert(isoc_temp(pt) == sbt->isOccupied(pt));
            if(sbt->getInternalBlockPtr(pt) != nullptr) {
                assert(isoc_temp(pt) == false);
            }
        }
        // debug: check whether every block cover all node in its range
        std::vector<bool> checked(total_index, false);
        for(Id id=0; id<total_index; id++) {
            if (checked[id]) { continue; }
            // assert every block is not out of map
            Pointi<N> pt = IdToPointi<N>(id, temp_dim);
            if (sbt->getInternalBlockPtr(pt) == nullptr) { continue; }
            auto node = sbt->getInternalBlockPtr(pt);

            Pointi<N> offset = node->max_ - node->min_;

            DimensionLength local_dim[N];
            for (int d = 0; d < N; d++) { local_dim[d] = offset[d] + 1; }
            Id local_total_index = getTotalIndexOfSpace<N>(local_dim), global_id;
            Pointi<N> local_pt, global_pt;
            for(Id id=0; id<local_total_index; id++) {
                local_pt = IdToPointi<N>(id, local_dim);
                global_pt = node->min_ + local_pt;
                // no grid in block is out of map
                assert(!isOutOfBoundary(global_pt, temp_dim));
                global_id = PointiToId<N>(global_pt, temp_dim);
                // all node in the block have the same block
                if(sbt->getInternalBlockPtr(global_pt) != node) {
                    std::cout << "sbt->getInternalBlockPtr(global_pt) = " << sbt->getInternalBlockPtr(global_pt) << std::endl;
                    std::cout << "node = " << node << std::endl;
                }
                assert(sbt->getInternalBlockPtr(global_pt) == node);
                assert(checked[global_id] == false);
            }
        }
        gettimeofday(&tv_after, &tz);
        double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;

        std::cout << "finish SpaceBinaryTree varify in " << time_cost << " ms" << std::endl;
    }

    // times_of_test do how many times of LOS compare
    // each compare use how many times of sample to find a point
    template<Dimension N>
    void LOSCompare(DimensionLength* temp_dim,
                    const SpaceBinaryTreePtr<N>& sbt,
                    const DynamicObstacles<N>& dynamic_obstacles,
                    std::vector<std::string>& output_strings,
                    int times_of_test = 1e5,
                    int max_sample_times = 1e3) {

        //SpaceBinaryTreeVarify<N>(temp_dim, dynamic_obstacles.isoc_, sbt);

        struct timezone tz;
        struct timeval tv_pre;
        struct timeval tv_after;

        Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();

        Id total_index = getTotalIndexOfSpace<N>(temp_dim);

        double sum_1 = 0, sum_2 = 0;
        int success_count = 0, occ_count = 0;
        std::cout << "haha times_of_test  = " << times_of_test << std::endl;
        for(int i=0; i<times_of_test; i++) {
            // random pick two passable point
            Id id1 = 0, id2 = 0;
            Pointi<N> pt1, pt2;
            int count = max_sample_times;
            while(count >= 0) {
                id1 = rand() % total_index;
                pt1 = IdToPointi<N>(id1, temp_dim);
                if (!dynamic_obstacles.isoc_(pt1)) {
                    break;
                } else {
                    count --;
                }
            }
            if (dynamic_obstacles.isoc_(pt1)) {
                continue;
            }
            count = max_sample_times;
            while(count >= 0) {
                id2 = rand() % total_index;
                pt2 = IdToPointi<N>(id2, temp_dim);
                if (!dynamic_obstacles.isoc_(pt2)) {
                    break;
                } else {
                    count --;
                }
            }
            if (dynamic_obstacles.isoc_(pt2)) {
                continue;
            }

            sbt->raw_visited_pt_count_ = 0, sbt->SBT_visited_pt_count_ = 0;

            //std::cout << "do LOS between " << pt1 << ", " << pt2 <<  std::endl;
            gettimeofday(&tv_pre, &tz);
            bool isoc1 = sbt->lineCrossObstacleRaw(pt1, pt2, dynamic_obstacles.isoc_);

            gettimeofday(&tv_after, &tz);
            double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e6 + (tv_after.tv_usec - tv_pre.tv_usec);
            sum_1 = sum_1 + time_cost1;
            Pointis<N> visited_pt;
            int count_of_block;
            gettimeofday(&tv_pre, &tz);
            bool isoc2 = sbt->lineCrossObstacleSBT(pt1, pt2, dynamic_obstacles.isoc_, visited_pt, count_of_block);
            //bool isoc2 = sbt->lineCrossObstacleSBT(pt1, pt2, dynamic_obstacles.isoc_);

            gettimeofday(&tv_after, &tz);
            double time_cost2 = (tv_after.tv_sec - tv_pre.tv_sec)*1e6 + (tv_after.tv_usec - tv_pre.tv_usec);
            sum_2 = sum_2 + time_cost2;
            success_count ++;

            //std::cout << "raw / SBT visited pt = " << sbt->raw_visited_pt_count_ << " / " << sbt->SBT_visited_pt_count_ << std::endl;


            if(isoc1 != isoc2)
            {
                Pointis<2> pts_raw;

                std::cout << "Raw LOS visited_pt = ";
                Line<N> line(pt1, pt2);
                int check_step = line.step;
                Pointi<N> pt, occ_pt;
                for(int i=1; i<check_step; i++) {
                    pt = line.GetPoint(i);
                    std::cout << pt << "(" << dynamic_obstacles.isoc_(pt) << ")， ";
                    if(dynamic_obstacles.isoc_(pt)) {
                        occ_pt = pt;
                        break;
                    }
                }
                std::cout << std::endl;

                std::cout << i << " th test failed, pt1/pt2 = " << pt1 << " / " << pt2 << std::endl;
                std::cout << "raw LOS = " << isoc1 << ", SBT LOS = " << isoc2 << std::endl;

                std::cout << "SBT visited_pt = ";

                for(const auto& vpt : visited_pt) {
                    std::cout << vpt << "(" << dynamic_obstacles.isoc_(vpt) << "), ";
                }
                std::cout << std::endl;

                Id occ_id = PointiToId(occ_pt, temp_dim);
                std::cout << "occ_pt in block = " << sbt->getInternalBlockPtr(occ_pt) << std::endl;

                std::cout << "dim info " << printDimInfo<N>(temp_dim) << std::endl;
                std::cout << "dynamic obstacles = " << dynamic_obstacles << std::endl;
                // assert classic LOS check and SBT's LOS check have the same result
                assert(isoc1 == isoc2);
            }

            if(isoc1 == true) { occ_count ++; }

            std::stringstream ss;
            ss << dynamic_obstacles.occ_pt_count_ << " "
               << getTotalIndexOfSpace<N>(temp_dim) << " "
               << time_cost1 << " "
               << time_cost2;
            output_strings.push_back(ss.str());

        }
        std::cout << "haha" << std::endl;
        Id space_size = getTotalIndexOfSpace<N>(temp_dim);
        std::cout << "dim info " << printDimInfo<N>(temp_dim) << std::endl;
        std::cout << "dynamic obstacles " << dynamic_obstacles << std::endl;
        std::cout << "obstacleDensity " << (float)dynamic_obstacles.occ_pt_count_ / space_size << std::endl;
        std::cout << success_count <<  " LOS test, mean raw / SBT LOS time cost (us) = " << sum_1/(double)success_count
                  << " / " << sum_2/(double)success_count << std::endl;
        std::cout << "occ ratio = " << (float)occ_count / success_count << std::endl;
    }


    // times_of_test do how many times of LOS compare
    // each compare use how many times of sample to find a point
    template<Dimension N>
    void LOSCompare(DimensionLength* temp_dim,
                    const IS_OCCUPIED_FUNC<N>& isoc_temp,
                    const SpaceBinaryTreePtr<N>& sbt,
                    int times_of_test = 1e6,
                    int max_sample_times = 1e3) {

        //SpaceBinaryTreeVarify<N>(temp_dim, isoc_temp, sbt);

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
            //sbt->raw_visited_pt_count_ = 0, sbt->SBT_visited_pt_count_ = 0;

            gettimeofday(&tv_pre, &tz);
            bool isoc1 = sbt->lineCrossObstacleRaw(pt1, pt2, isoc_temp);
            gettimeofday(&tv_after, &tz);
            double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e6 + (tv_after.tv_usec - tv_pre.tv_usec);
            sum_1 = sum_1 + time_cost1;
            Pointis<N> visited_pt;
            int count_of_block;
            gettimeofday(&tv_pre, &tz);
            bool isoc2 = sbt->lineCrossObstacleSBT(pt1, pt2, isoc_temp, visited_pt, count_of_block);
            gettimeofday(&tv_after, &tz);
            double time_cost2 = (tv_after.tv_sec - tv_pre.tv_sec)*1e6 + (tv_after.tv_usec - tv_pre.tv_usec);
            sum_2 = sum_2 + time_cost2;
            success_count ++;

            //std::cout << "raw / SBT visited pt = " << sbt->raw_visited_pt_count_ << " / " << sbt->SBT_visited_pt_count_ << std::endl;


            if(isoc1 != isoc2)
            {
                Pointis<2> pts_raw;

                std::cout << "Raw LOS visited_pt = ";
                Line<N> line(pt1, pt2);
                int check_step = line.step;
                Pointi<N> pt;
                for(int i=1; i<check_step; i++) {
                    pt = line.GetPoint(i);
                    std::cout << pt << "(" << isoc_temp(pt) << ")， ";
                }
                std::cout << std::endl;

                std::cout << i << " th test failed, pt1/pt2 = " << pt1 << " / " << pt2 << std::endl;
                std::cout << "raw LOS = " << isoc1 << ", SBT LOS = " << isoc2 << std::endl;

                std::cout << "SBT visited_pt = ";

                for(const auto& vpt : visited_pt) {
                    std::cout << vpt << "(" << isoc_temp(vpt) << "), ";
                }
                std::cout << std::endl;
                // assert classic LOS check and SBT's LOS check have the same result
                assert(isoc1 == isoc2);
            }
        }

        std::cout << success_count <<  " LOS test, mean raw / SBT LOS time cost (us) = " << sum_1/(double)success_count
                  << " / " << sum_2/(double)success_count << std::endl;

    }

    template<Dimension N>
    ObstaclePtrs<N> generateRandomObstacles(const int& number_of_obstacle,
                                 int min_radius = 5,
                                 int max_radius = 10,
                                 int min_block_width = 10,
                                 int max_block_width = 20) {
        ObstaclePtrs<N> obs;

        CircleObstaclePtrs<N> co = generateRandomCircleObstacles<N>(number_of_obstacle/2, min_radius, max_radius);
        obs.insert(obs.end(), co.begin(), co.end());

        Pointi<N> min_pt, max_pt;
        min_pt.setAll(min_block_width);
        max_pt.setAll(max_block_width);
        BlockObstaclePtrs<N> bo = generateRandomBlockObstacles<N>(number_of_obstacle/2, min_pt, max_pt);
        obs.insert(obs.end(), co.begin(), co.end());

        return obs;
    }

    // half obstacle is circle and another half is block obstacle
    template<Dimension N, typename SBTtype>
    void massiveSBTLOSCompareTest(int random_times, // how many times of randomize for a map
                                  int repeat_times, // how many times of LOS for a randomize
                                  const std::vector<int>& width_of_space,
                                  const std::vector<int>& number_of_obstacles,
                                  int time_of_test = 1e6,
                                  int max_sample_times = 1e3) {

        struct timezone tz;
        struct timeval tv_pre;
        struct timeval tv_after;


        for(const auto& width : width_of_space) {
            for(const auto& count : number_of_obstacles) {

                // debug: do not use external config of obstacles
                int local_min_radius = width/20,
                    local_max_radius = width/10,
                    local_min_block_width = width/10,
                    local_max_block_width = width/5;

                ObstaclePtrs<N> obs = generateRandomObstacles<N>(count,
                                                                 local_min_radius,
                                                                 local_max_radius,
                                                                 local_min_block_width,
                                                                 local_max_block_width);

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

                gettimeofday(&tv_pre, &tz);

                SpaceBinaryTreePtr<N> sbt = std::make_shared<SBTtype>(is_occupied, dim);
                sbt->initialize();

                gettimeofday(&tv_after, &tz);
                double time_cost_init = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
                std::cout << "SBT init time cost " << time_cost_init << " ms" << std::endl;

                DynamicObstacles<N> dynamic_obstacles(dim, obs);

                Id total_index = getTotalIndexOfSpace<N>(dim);
                std::cout << N << " dimension space, width = " << width << ", number of obstacles = " << count << std::endl;

                for(int i=0; i<random_times; i++) {

                    dynamic_obstacles.random();

                    gettimeofday(&tv_pre, &tz);

                    for(const auto& pre_pt : dynamic_obstacles.getPreviousOccupationPoints()) {
                        sbt->setOccupiedState(pre_pt, false);
                    }
                    for(const auto& cur_pt : dynamic_obstacles.getCurrentOccupationPoints()) {
                        sbt->setOccupiedState(cur_pt, true);
                    }
                    gettimeofday(&tv_after, &tz);
                    double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e6 + (tv_after.tv_usec - tv_pre.tv_usec);

                    std::cout << "dynamicUpdateTimeCost " << time_cost1 << " us" << std::endl;

                    std::vector<std::string> ss;
                    LOSCompare<N>(dim, sbt, dynamic_obstacles, ss, time_of_test, max_sample_times);

                }

            }
        }
    }

}

#endif //JUMPOVERBLOCK_DEPENDENCIES_H
