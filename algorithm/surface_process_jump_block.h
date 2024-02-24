//
// Created by yaozhuo on 2023/5/17.
//

#ifndef FREENAV_SURFACE_PROCESS_JUMP_BLOCK_H
#define FREENAV_SURFACE_PROCESS_JUMP_BLOCK_H

#include "../freeNav-base/basic_elements/surface_process.h"
#include "../algorithm/line_of_sight_jump_between_block.h"
#include "../algorithm/block_detect.h"
#include "../algorithm/block_detector_greedy.h"

namespace freeNav::JOB {



//    template<Dimension N>
//    int findExitPointOfBlock(Line<N>& line, const Pointi<N>& current_pt, int& index, const BlockPtr<N>& block_ptr);




    template <Dimension N>
    class SurfaceProcessorSparseWithJumpBlock : public SurfaceProcessorSparse<N> {
    public:
        SurfaceProcessorSparseWithJumpBlock(DimensionLength* dimension_info,
                                            IS_OCCUPIED_FUNC<N> is_occupied,
                                            SET_OCCUPIED_FUNC<N> set_occupied,
                                            const std::vector<Pointi<N> >& occ_grids,
                                            const IdSet& occ_voxel_ids,
                                            PathLen minimum_block_width = 10,
                                            std::string block_file_path = "",
                                            bool force_update = false
        ) : SurfaceProcessorSparse<N>(dimension_info, is_occupied, set_occupied, occ_grids, occ_voxel_ids)  {
            struct timeval tv_pre, tv_after;
            struct timezone tz;
            gettimeofday(&tv_pre, &tz);
            this->surfaceGridsDetection(true);
            block_detector_ = std::make_shared<BlockDetector<N> >(dimension_info,
                                                                  is_occupied,
                                                                  this->getSurfaceGrids(),
                                                                  minimum_block_width,
                                                                  block_file_path,
                                                                  force_update);
            gettimeofday(&tv_after, &tz);
            double block_detect_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            std::cout << "-- detect " << block_detector_->all_block_ptrs_.size() << " block in " << block_detect_cost << "ms" << std::endl << std::endl;
        }

        bool lineCrossObstacle(const Pointi<N>& pt1, const Pointi<N>& pt2) override {
            Pointis<N> visited_pt;
            int count_of_block;
            return LineCrossObstacleWithBlockJump(pt1, pt2, (BlockDetectorInterfacePtr<N>)block_detector_, visited_pt, count_of_block);
        }

        bool lineCrossObstacleWithVisitedPoint(const Pointi<N>& pt1, const Pointi<N>& pt2, Pointis<N>& visited_pt, int& count_of_block) {
            visited_pt.clear();
            return LineCrossObstacleWithBlockJump(pt1, pt2, (BlockDetectorInterfacePtr<N>)block_detector_, visited_pt, count_of_block);
        }


        BlockDetectorPtr<N> block_detector_;

    };

    template <Dimension N>
    class SurfaceProcessorSparseWithJumpBlockGreedy : public SurfaceProcessorSparse<N> {
    public:
        SurfaceProcessorSparseWithJumpBlockGreedy(DimensionLength* dimension_info,
                                                 IS_OCCUPIED_FUNC<N> is_occupied,
                                                 SET_OCCUPIED_FUNC<N> set_occupied,
                                                 const std::vector<Pointi<N> >& occ_grids,
                                                 const IdSet& occ_voxel_ids,
                                                 PathLen minimum_block_width = 10,
                                                 std::string block_file_path = "",
                                                 bool force_update = false
        ) : SurfaceProcessorSparse<N>(dimension_info, is_occupied, set_occupied, occ_grids, occ_voxel_ids) {
            struct timeval tv_pre, tv_after;
            struct timezone tz;
            gettimeofday(&tv_pre, &tz);
            this->surfaceGridsDetection(true);
            block_detector_ = std::make_shared<BlockDetectorGreedy<N> >(dimension_info,
                                                                        is_occupied,
                                                                        this->getSurfacePts(),
                                                                        minimum_block_width,
                                                                        block_file_path,
                                                                        force_update);
            gettimeofday(&tv_after, &tz);
            double block_detect_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            std::cout << "-- detect " << block_detector_->all_block_ptrs_.size() << " block in " << block_detect_cost << "ms" << std::endl << std::endl;
        }

        bool lineCrossObstacle(const Pointi<N>& pt1, const Pointi<N>& pt2) override {
            Pointis<N> visited_pt;
            int count_of_block;
            return LineCrossObstacleWithBlockJump(pt1, pt2, (BlockDetectorInterfacePtr<N>)block_detector_, visited_pt, count_of_block);
        }

        bool lineCrossObstacleWithVisitedPoint(const Pointi<N>& pt1, const Pointi<N>& pt2, Pointis<N>& visited_pt, int& count_of_block) {
            visited_pt.clear();
            return LineCrossObstacleWithBlockJump(pt1, pt2, (BlockDetectorInterfacePtr<N>)block_detector_, visited_pt, count_of_block);
        }


        BlockDetectorGreedyPtr<N> block_detector_;

    };

    template <Dimension N>
    class SurfaceProcessorSparseWithJumpBlockGreedyShrink : public SurfaceProcessorSparse<N> {
    public:
        SurfaceProcessorSparseWithJumpBlockGreedyShrink(DimensionLength* dimension_info,
                                                        IS_OCCUPIED_FUNC<N> is_occupied,
                                                        SET_OCCUPIED_FUNC<N> set_occupied,
                                                        const Pointis<N>& occ_grids,
                                                        const IdSet& occ_voxel_ids,
                                                        int shrink_level = 5,
                                                        PathLen minimum_block_width = 10,
                                                        std::string block_file_path = "",
                                                        bool force_update = false
        ) : SurfaceProcessorSparse<N>(dimension_info, is_occupied, set_occupied, occ_grids, occ_voxel_ids) {
            struct timeval tv_pre, tv_after;
            struct timezone tz;
            gettimeofday(&tv_pre, &tz);
            this->surfaceGridsDetection(true);
            block_detector_ = std::make_shared<BlockDetectorGreedyWithShrink<N> >(dimension_info,
                                                                                  is_occupied,
                                                                                  occ_grids,
                                                                                  shrink_level,
                                                                                  minimum_block_width,
                                                                                  block_file_path,
                                                                                  force_update);
            gettimeofday(&tv_after, &tz);
            double block_detect_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            std::cout << "-- detect " << block_detector_->all_block_ptrs_.size() << " block in " << block_detect_cost << "ms" << std::endl << std::endl;
        }

        bool lineCrossObstacle(const Pointi<N>& pt1, const Pointi<N>& pt2) override {
            Pointis<N> visited_pt;
            int count_of_block;
            return LineCrossObstacleWithBlockJump(pt1, pt2, (BlockDetectorInterfacePtr<N>)block_detector_, visited_pt, count_of_block);
        }

        bool lineCrossObstacleWithVisitedPoint(const Pointi<N>& pt1, const Pointi<N>& pt2, Pointis<N>& visited_pt, int& count_of_block) {
            visited_pt.clear();
            return LineCrossObstacleWithBlockJump(pt1, pt2, (BlockDetectorInterfacePtr<N>)block_detector_, visited_pt, count_of_block);
        }

        BlockDetectorGreedyWithShrinkPtr<N> block_detector_;

        // TODO: considering shrink inside space rather than outside of space
        //BlockDetectorGreedyPtr<N> block_detector_shrink_;

    };

    template <Dimension N>
    using SurfaceProcessorSparseWithJumpBlockGreedyShrinkPtr = std::shared_ptr<SurfaceProcessorSparseWithJumpBlockGreedyShrink<N> >;

    class SurfaceProcessorSparseWithJumpBlockOctoMap : public SurfaceProcessorSparse<3> {
    public:
        SurfaceProcessorSparseWithJumpBlockOctoMap(DimensionLength* dimension_info,
                                                   IS_OCCUPIED_FUNC<3> is_occupied,
                                                   SET_OCCUPIED_FUNC<3> set_occupied,
                                                   const std::vector<Pointi<3> >& occ_grids,
                                                   const IdSet& occ_voxel_ids,
                                                   PathLen minimum_block_width = 10,
                                                   std::string block_file_path = "",
                                                   bool force_update = false
        ) : SurfaceProcessorSparse<3>(dimension_info, is_occupied, set_occupied, occ_grids, occ_voxel_ids) {
            struct timeval tv_pre, tv_after;
            struct timezone tz;
            gettimeofday(&tv_pre, &tz);
            this->surfaceGridsDetection(true);
            block_detector_ = std::make_shared<BlockDetectorOctoMap>(dimension_info,
                                                                     is_occupied,
                                                                     this->getSurfaceGrids(),
                                                                     occ_grids,
                                                                     minimum_block_width,
                                                                     block_file_path,
                                                                     force_update);
            gettimeofday(&tv_after, &tz);
            double block_detect_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            std::cout << "-- detect " << block_detector_->all_block_ptrs_.size() << " block in " << block_detect_cost << "ms" << std::endl << std::endl;
        }

        bool lineCrossObstacle(const Pointi<3>& pt1, const Pointi<3>& pt2) override {
            Pointis<3> visited_pt;
            int count_of_block;
            return LineCrossObstacleWithBlockJump(pt1, pt2, (BlockDetectorInterfacePtr<3>)block_detector_, visited_pt, count_of_block);
        }

        bool lineCrossObstacleWithVisitedPoint(const Pointi<3>& pt1, const Pointi<3>& pt2, Pointis<3>& visited_pt, int& count_of_block) {
            visited_pt.clear();
            return LineCrossObstacleWithBlockJump(pt1, pt2, (BlockDetectorInterfacePtr<3>)block_detector_, visited_pt, count_of_block);
        }


        BlockDetectorOctoMapPtr block_detector_;

    };

    template <Dimension N>
    using SurfaceProcessorSparseWithJumpBlockOctoMapPtr = std::shared_ptr<SurfaceProcessorSparseWithJumpBlockOctoMap>;

}

#endif //FREENAV_SURFACE_PROCESS_JUMP_BLOCK_H
