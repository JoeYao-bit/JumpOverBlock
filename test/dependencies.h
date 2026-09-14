//
// Created by yaozhuo on 2025/4/29.
//

#ifndef JUMPOVERBLOCK_DEPENDENCIES_H
#define JUMPOVERBLOCK_DEPENDENCIES_H

#include "../algorithm/space_binary_tree/space_binary_tree_raw.h"
#include "../algorithm/space_binary_tree/space_binary_tree_shrink.h"

#include "dynamic_obstacles.h"
#include <fstream>

namespace freeNav::JOB {

    template<Dimension N>
    void writeToFile(const std::vector<std::string>& strs, const std::string& file_path) {
        std::ofstream os(file_path, std::ios::app);
        if(!os.is_open()) {
            std::cout << "file_path " << file_path << " open failed" << std::endl;
            return;
        }
        for(const auto& str : strs) {
            os << str << std::endl;
        }
        os.close();
    }

    template<Dimension N, typename SBT>
    void SpaceBinaryTreeVarify(DimensionLength* temp_dim,
                               const IS_OCCUPIED_FUNC<N>& isoc_temp,
                               const std::shared_ptr<SBT>& sbt) {

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
            assert(isoc_temp(pt) == sbt->isoc_dynamic_(pt));
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


    template<Dimension N>
    std::vector<std::pair<Pointi<N>, Pointi<N>>> getLOSTestCases(DimensionLength* dim,
                                                                 const IS_OCCUPIED_FUNC<N>& isoc,
                                                                 int times_of_test = 1e5,
                                                                 int max_sample_times = 1e3) {
        Id total_index = getTotalIndexOfSpace<N>(dim);
        std::vector<std::pair<Pointi<N>, Pointi<N>>> retv;
        for(int i=0; i<times_of_test; i++) {
            // random pick two passable point
            Id id1 = 0, id2 = 0;
            Pointi<N> pt1, pt2;
            int count = max_sample_times;
            while(count >= 0) {
                id1 = rand() % total_index;
                pt1 = IdToPointi<N>(id1, dim);
                if (!isoc(pt1)) {
                    break;
                } else {
                    count --;
                }
            }
            if (isoc(pt1)) {
                continue;
            }
            count = max_sample_times;
            while(count >= 0) {
                id2 = rand() % total_index;
                pt2 = IdToPointi<N>(id2, dim);
                if (!isoc(pt1)) {
                    break;
                } else {
                        count --;
                    }
            }
            if (isoc(pt2)) {
                continue;
            }
            //std::cout << "do LOS between " << pt1 << ", " << pt2 <<  std::endl;
            retv.push_back({pt1, pt2});
        }
        return retv;
    }

    // times_of_test do how many times of LOS compare
    // each compare use how many times of sample to find a point
    template<Dimension N, typename SBT>
    void LOSCompare(DimensionLength* temp_dim,
                    const std::shared_ptr<SBT>& sbt,
                    std::vector<std::string>& output_strings,
                    const std::string& identifier,
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
        std::cout << "times_of_LOS_compare_test  = " << times_of_test << std::endl;
        for(int i=0; i<times_of_test; i++) {
            // random pick two passable point
            Id id1 = 0, id2 = 0;
            Pointi<N> pt1, pt2;
            int count = max_sample_times;
            while(count >= 0) {
                id1 = rand() % total_index;
                pt1 = IdToPointi<N>(id1, temp_dim);
                if (!sbt->isoc_dynamic_(pt1)) {
                    break;
                } else {
                    count --;
                }
            }
            if (sbt->isoc_dynamic_(pt1)) {
                continue;
            }
            count = max_sample_times;
            while(count >= 0) {
                id2 = rand() % total_index;
                pt2 = IdToPointi<N>(id2, temp_dim);
                if (!sbt->isoc_dynamic_(pt1)) {
                    break;
                } else {
                    count --;
                }
            }
            if (sbt->isoc_dynamic_(pt2)) {
                continue;
            }

            //sbt->raw_visited_pt_count_ = 0, sbt->SBT_visited_pt_count_ = 0;

            //std::cout << "do LOS between " << pt1 << ", " << pt2 <<  std::endl;
            gettimeofday(&tv_pre, &tz);
            bool isoc1 = sbt->lineCrossObstacleRaw(pt1, pt2, sbt->isoc_dynamic_);

            gettimeofday(&tv_after, &tz);
            double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e6 + (tv_after.tv_usec - tv_pre.tv_usec);
            sum_1 = sum_1 + time_cost1;
            Pointis<N> visited_pt;
            int count_of_block;
            gettimeofday(&tv_pre, &tz);
            bool isoc2 = sbt->lineCrossObstacleSBT(pt1, pt2, sbt->isoc_dynamic_, count_of_block);
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
                Line<int, N> line(pt1, pt2);
                int check_step = line.step;
                Pointi<N> pt, occ_pt;
                for(int i=1; i<check_step; i++) {
                    pt = line.GetPoint(i);
                    std::cout << pt << "(" << sbt->isoc_dynamic_(pt) << ")， ";
                    if(sbt->isoc_dynamic_(pt)) {
                        occ_pt = pt;
                        break;
                    }
                }
                std::cout << std::endl;

                std::cout << i << " th test failed, pt1/pt2 = " << pt1 << " / " << pt2 << std::endl;
                std::cout << "raw LOS = " << isoc1 << ", SBT LOS = " << isoc2 << std::endl;

                std::cout << "SBT visited_pt = ";

                for(const auto& vpt : visited_pt) {
                    std::cout << vpt << "(" << sbt->isoc_dynamic_(vpt) << "), ";
                }
                std::cout << std::endl;

                Id occ_id = PointiToId(occ_pt, temp_dim);
                std::cout << "occ_pt in block = " << sbt->getInternalBlockPtr(occ_pt) << std::endl;

                std::cout << "dim info " << printDimInfo<N>(temp_dim) << std::endl;
                // assert classic LOS check and SBT's LOS check have the same result
                assert(isoc1 == isoc2);
            }

            if(isoc1 == true) { occ_count ++; }

        }
        Id space_size = getTotalIndexOfSpace<N>(temp_dim);
        float obstacleDensity = sbt->getObstacleDensity();
        std::cout << identifier << " dim info " << printDimInfo<N>(temp_dim) << std::endl;
        std::cout << identifier << " obstacleDensity " << obstacleDensity << std::endl;
        std::cout << identifier << " " << success_count <<  " LOS test, mean raw / SBT LOS time cost (us) = " << sum_1/(double)success_count
                  << " / " << sum_2/(double)success_count << std::endl;
        std::cout << identifier << " occ_ratio(LOS_pass/LOS_total) = " << (float)occ_count / success_count << std::endl;
        std::cout << identifier << " SBT / raw visit pt count = " << sbt->SBT_visited_pt_count_
                  << " / " << sbt->raw_visited_pt_count_ << " = "
                  << ((float)sbt->SBT_visited_pt_count_)/sbt->raw_visited_pt_count_ << std::endl;

        std::stringstream ss;
        ss << "COMPARE_" << identifier << " " << N << " "  // Dimension
           << (float)occ_count / success_count << " " // occ ratio
           << sum_1/(double)success_count << " " // mean raw time cost
           << sum_2/(double)success_count << " " // mean SBT time cost
           << getTotalIndexOfSpace<N>(temp_dim) << " " // total index of space
           << obstacleDensity << " " // ratio of occ grid
           << printDimInfo<N>(temp_dim) << " " // dimension length
           ;
        output_strings.push_back(ss.str());

    }


    // times_of_test do how many times of LOS compare
    // each compare use how many times of sample to find a point
    template<Dimension N>
    void LOSCompare(DimensionLength* temp_dim,
                    const IS_OCCUPIED_FUNC<N>& isoc_temp,
                    const SpaceBinaryTreeRawPtr<N>& sbt,
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
            bool isoc2 = sbt->lineCrossObstacleSBT(pt1, pt2, isoc_temp, count_of_block);
            gettimeofday(&tv_after, &tz);
            double time_cost2 = (tv_after.tv_sec - tv_pre.tv_sec)*1e6 + (tv_after.tv_usec - tv_pre.tv_usec);
            sum_2 = sum_2 + time_cost2;
            success_count ++;

            //std::cout << "raw / SBT visited pt = " << sbt->raw_visited_pt_count_ << " / " << sbt->SBT_visited_pt_count_ << std::endl;


            if(isoc1 != isoc2)
            {
                Pointis<2> pts_raw;

                std::cout << "Raw LOS visited_pt = ";
                Line<int, N> line(pt1, pt2);
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
        obs.insert(obs.end(), bo.begin(), bo.end());

        return obs;
    }

    // half obstacle is circle and another half is block obstacle
    template<Dimension N>
    void massiveSBTLOSCompareTest(int random_times, // how many times of randomize for a map
                                  int repeat_times, // how many times of LOS for a randomize
                                  const std::vector<int>& width_of_space,
                                  const std::vector<int>& number_of_obstacles,
                                  const std::string& file_path,
                                  int time_of_test = 1e6,
                                  int max_sample_times = 1e3,
                                  const std::vector<int>& max_obs_move_distances = {10},
                                  bool update_block_ptr_realtime = true,
                                  int  min_block_depth_width = 4) {

        for(const auto& width : width_of_space) {
            for(const auto& count : number_of_obstacles) {
                for(const auto& max_obs_move_distance : max_obs_move_distances) {
                    // debug: do not use external config of obstacles
                    int local_min_radius = width / 20,
                            local_max_radius = width / 10,
                            local_min_block_width = width / 20,
                            local_max_block_width = width / 10;

                    ObstaclePtrs<N> obs = generateRandomObstacles<N>(count,
                                                                     local_min_radius,
                                                                     local_max_radius,
                                                                     local_min_block_width,
                                                                     local_max_block_width);

                    DimensionLength dim[N];
                    for (int d = 0; d < N; d++) {
                        dim[d] = width;
                    }
                    auto is_occupied = [&](const Pointi<N> &pt) -> bool {
                        if (isOutOfBoundary(pt, dim)) {
                            return true;
                        }
                        return false;
                    };

                    USTimer mst;

                    DynamicObstacles<N> dynamic_obstacles(dim, obs);
                    Id total_index = getTotalIndexOfSpace<N>(dim);
//                    std::cout << N << " dimension space, width = " << width << ", number of obstacles = " << count
//                              << std::endl;
//                    std::cout << "max_obs_move_distance = " << max_obs_move_distance << std::endl;

                    SpaceBinaryTreeRawPtr<N> sbt_raw =
                            std::make_shared<SpaceBinaryTreeAnyDimensionRaw<N> >(dynamic_obstacles.isoc_, dim,
                                                                          1);
                    sbt_raw->initialize();

                    SpaceBinaryTreeShrinkPtr<N> sbt =
                            std::make_shared<SpaceBinaryTreeShrink<N> >(dynamic_obstacles.isoc_, dim,
                                                                              min_block_depth_width);
//                    sbt->initialize();
                    dynamic_obstacles.random();

                    for (int i = 0; i < random_times; i++) {

                        dynamic_obstacles.random(max_obs_move_distance);

                        mst.reset();
//                        // only update changed node
//                        for (const auto &new_free : dynamic_obstacles.getNewPassablePoints()) {
//                            sbt_raw->setOccupiedState(new_free, false, update_block_ptr_realtime);
//                        }
//                        for (const auto &new_occ : dynamic_obstacles.getNewOccupiedPoints()) {
//                            sbt_raw->setOccupiedState(new_occ, true, update_block_ptr_realtime);
//                        }
//                        if (!update_block_ptr_realtime) {
//                            sbt_raw->initBlockPtrMap();
//                        }
                        sbt_raw->setNewOccAndPassablePts(dynamic_obstacles.getNewPassablePoints(),
                                                         dynamic_obstacles.getNewOccupiedPoints());
                        double time_cost_update_raw = mst.elapsed()/1e3;

                        //std::cout << "raw dynamicUpdateTimeCost " << time_cost_update_raw << " ms" << std::endl;
//                    SpaceBinaryTreeVarify(dim, dynamic_obstacles.isoc_, sbt);
                        mst.reset();
                        SpaceBinaryTreeRawPtr<N> temp_sbt_raw =
                                std::make_shared<SpaceBinaryTreeAnyDimensionRaw<N>>(dynamic_obstacles.isoc_, dim,
                                                                                   1);
                        temp_sbt_raw->initialize();

                        double time_cost_init_raw = mst.elapsed()/1e3;
                        //std::cout << "raw SBT_init_time_cost " << time_cost_init_raw << " ms" << std::endl;


                        mst.reset();
                        sbt->setNewOccAndPassablePts(dynamic_obstacles.getNewPassablePoints(),
                                                     dynamic_obstacles.getNewOccupiedPoints());
                        double time_cost_update_new = mst.elapsed()/1e3;


//                    SpaceBinaryTreeVarify(dim, dynamic_obstacles.isoc_, sbt);
                        mst.reset();
                        SpaceBinaryTreeShrinkPtr<N> temp_sbt =
                                std::make_shared<SpaceBinaryTreeShrink<N> >(dynamic_obstacles.isoc_, dim,
                                                                              min_block_depth_width);
//                        temp_sbt->initialize();
                        double time_cost_init_new = mst.elapsed()/1e3;
                        //std::cout << "new SBT_init_time_cost " << time_cost_init_new << " ms" << std::endl;


                        std::cout << "raw SBT / new SBT  init  time cost compare = "
                                  << time_cost_init_raw << " / "
                                  << time_cost_init_new
                                  << std::endl;

                        std::cout << "raw SBT / new SBT update time cost compare = "
                                  << time_cost_update_raw << " / "
                                  << time_cost_update_new
                                  << std::endl;

//                        if (!file_path.empty()) {
//                            writeToFile<N>(strs, file_path);
//                            std::cout << "write test data to " << file_path << std::endl;
//                        }
                        int success_count = 0, occ_count = 0;
                        auto test_cases = getLOSTestCases<N>(dim, sbt->isoc_dynamic_, repeat_times);
                        std::cout << "times_of_LOS_compare_test  = " << test_cases.size() << std::endl;
                        if(test_cases.empty()) { continue; }
                        double sum_1 = 0, sum_2 = 0, sum_3 = 0;
                        int sum_count_of_block_1 = 0, sum_count_of_block_2 = 0;
                        USTimer ust;
                        sbt_raw->raw_visited_pt_count_ = 0;
                        sbt_raw->SBT_visited_pt_count_ = 0;

                        sbt->raw_visited_pt_count_ = 0;
                        sbt->SBT_visited_pt_count_ = 0;
                        for(const auto& test_case : test_cases) {
                            const auto& pt1 = test_case.first;
                            const auto& pt2 = test_case.second;
                            ust.reset();
                            bool isoc  = sbt_raw->lineCrossObstacleRaw(pt1, pt2, sbt_raw->isoc_dynamic_);
                            sum_1 = sum_1 + ust.elapsed();

                            ust.reset();
                            bool isoc1 = sbt_raw->lineCrossObstacleSBT(pt1, pt2, sbt_raw->isoc_dynamic_, sum_count_of_block_1);
                            sum_2 = sum_2 + ust.elapsed();

                            ust.reset();
                            bool isoc2 = sbt->lineCrossObstacleSBT(pt1, pt2, sbt_raw->isoc_dynamic_, sum_count_of_block_2);
                            sum_3 = sum_3 + ust.elapsed();

                            assert(isoc == isoc1);
                            assert(isoc == isoc2);

                            if(isoc) { occ_count ++; }
                            success_count++;

                        }

                        std::stringstream ss1;
                        ss1 << "SBT_RAW " << N << " "  // Dimension
                            << time_cost_init_raw << " " // init time cost
                            << time_cost_update_raw << " " // update time cost
                            << total_index << " " // total index of space
                            << (float)occ_count / success_count << " " // ratio of occ grid
                            << max_obs_move_distance << " "
                            << printDimInfo<N>(dim) << " " // dimension length
                                ;
                        std::vector<std::string> strs;
                        strs.push_back(ss1.str());

                        std::stringstream ss2;
                        ss2 << "SBT " << N << " "  // Dimension
                            << time_cost_init_new << " " // init time cost
                            << time_cost_update_new << " " // update time cost
                            << total_index << " " // total index of space
                            << (float)occ_count / success_count << " " // ratio of occ grid
                            << max_obs_move_distance << " "
                            << printDimInfo<N>(dim) << " " // dimension length
                                ;
                        strs.push_back(ss2.str());

                        float obstacleDensity_1 = sbt->getObstacleDensity();
//                        float obstacleDensity_2 = temp_sbt_raw->getObstacleDensity();
                        std::cout << "dim info " << printDimInfo<N>(dim) << std::endl;
                        std::cout << "max_obs_move_distance = " << max_obs_move_distance << std::endl;
                        std::cout << "random times = " << i << std::endl;
                        std::cout << "number_of_obstacle = " << count << std::endl;

                        std::cout << "obstacleDensity 1 " << obstacleDensity_1 << std::endl;
                        //std::cout << "obstacleDensity 2 " << obstacleDensity_2 << std::endl;
                        std::cout << "occ_ratio(LOS_pass/LOS_total) = " << (float)occ_count / success_count << std::endl;
                        std::cout << "new free pt size = " << dynamic_obstacles.getNewPassablePoints().size() << std::endl;
                        std::cout << "new occ pt size = " << dynamic_obstacles.getNewOccupiedPoints().size() << std::endl;
                        std::cout << success_count <<  " LOS test, mean raw / SBT / new SBT LOS time cost (us) = "
                                  << sum_1/(double)success_count << " / "
                                  << sum_2/(double)success_count << " / "
                                  << sum_3/(double)success_count << " / "
                                  << std::endl;

                        std::cout << "raw / raw SBT / new SBT compare with raw LOS visit pt count = "
                                  << (double)sbt_raw->raw_visited_pt_count_/success_count << " / "
                                  << (double)sbt_raw->SBT_visited_pt_count_/success_count << " / "
                                  << (double)sbt->SBT_visited_pt_count_/success_count  << " "
                                  << std::endl;

                        std::cout << "mean raw SBT / new SBT LOS visit block count = "
                                  << (double)sum_count_of_block_1/success_count << " / "
                                  << (double)sum_count_of_block_2/success_count << " "
                                  << std::endl;

                        std::stringstream ss3;
                        ss3 << "COMPARE" << " " << N << " "  // Dimension
                            << (float)occ_count / success_count << " " // occ ratio
                            << sum_1/success_count << " " // mean raw time cost
                            << sum_2/success_count << " " // mean raw SBT time cost
                            << sum_3/success_count << " " // mean new SBT time cost

                            << (double)sbt_raw->raw_visited_pt_count_/success_count << " " // mean raw visited pt
                            << (double)sbt_raw->SBT_visited_pt_count_/success_count << " " // mean raw SBT visited pt
                            << (double)sbt->SBT_visited_pt_count_/success_count << " " // mean new SBTvisited pt

                            << (double)sum_count_of_block_1/success_count << " " // mean raw SBT visited block
                            << (double)sum_count_of_block_2/success_count << " " // mean new SBT visited block

                            << getTotalIndexOfSpace<N>(dim) << " " // total index of space
                            << obstacleDensity_1 << " " // ratio of occ grid
                            << printDimInfo<N>(dim) << " " // dimension length
                                ;
                        strs.push_back(ss3.str());
                        if (!file_path.empty()) {
                            writeToFile<N>(strs, file_path);
                            std::cout << "write test data to " << file_path << std::endl;
                        }
                    }
                }
            }
        }
    }

    // if the line is collision free, return true, otherwise false
    template <Dimension N>
    using PATH_PLANNING_FUNC = std::function<Pointis<N>(const Pointi<N>&, const Pointi<N>&)>;

    template <Dimension N>
    using PATH_PLANNING_FUNC_WITH_LINE = std::function<Pointis<N>(const Pointi<N>&,
                                                                  const Pointi<N>&,
                                                                  const IS_LINE_COLLISION_FREE_FUNC<int, N>&,
                                                                  const std::string&)>;

}

#endif //JUMPOVERBLOCK_DEPENDENCIES_H
