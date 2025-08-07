//
// Created by yaozhuo on 8/5/25.
//

#ifndef JUMPOVERBLOCK_SPACE_BINARY_TREE_LAYERED_H
#define JUMPOVERBLOCK_SPACE_BINARY_TREE_LAYERED_H

#include "space_binary_tree_raw.h"

namespace freeNav::JOB {


    template<Dimension N>
    class SpaceBinaryTreeShrink {

    public:

        SpaceBinaryTreeShrink(const IS_OCCUPIED_FUNC<N>& isoc, DimensionLength* dim, int min_block_depth_width = 2) :
                dim_(dim), isoc_(isoc), min_block_depth_width_(min_block_depth_width) {

            // initialize
            // get max dimension length
            DimensionLength max_dim = 0;
            for (int i = 0; i < N; i++) {
                max_dim = std::max(max_dim, dim_[i]);
            }
//            std::cout << "max_dim_length = " << max_dim << std::endl;
            max_depth_ = 1;
            while (true) {
                if (pow(2, max_depth_) < max_dim) {
                    max_depth_++;
                } else {
                    break;
                }
            }

            // precomputation of pow(2, x)
            for (int dp = 0; dp <= max_depth_ * (int) N; dp++) {
                pow_2_.push_back(pow(2, dp));
            }


            raw_map_.resize(getTotalIndexOfSpace<N>(dim_), true);

            for(int i=0; i<raw_map_.size(); i++) {
                raw_map_[i] = isoc_(IdToPointi<N>(i, dim_));
            }

            local_dim_ = new DimensionLength[N];
            for(int d=0; d<N; d++) {
                local_dim_[d] = pow_2_[min_block_depth_width];
            }
            local_total_index_ = getTotalIndexOfSpace<N>(local_dim_);

            shrink_dim_ = new DimensionLength[N];
            for(int i=0 ;i<N; i++) {
                shrink_dim_[i] = pow_2_[max_depth_]/pow_2_[min_block_depth_width];
            }
            std::cout << "raw dim = " << printDimInfo<N>(dim_) << ", shrink dim = " << printDimInfo<N>(shrink_dim_) << std::endl;

            shrink_map_.resize(getTotalIndexOfSpace<N>(shrink_dim_), true);

            for(int i=0; i<shrink_map_.size(); i++) {
                Pointi<N> base_pt = IdToPointi<N>(i, shrink_dim_);
                bool is_occupied = false;
                //std::cout << "base pt = " << base_pt << ": ";
                for(int j=0; j<local_total_index_; j++) {
                    Pointi<N> pt = IdToPointi<N>(j, local_dim_);
                    Pointi<N> global_pt = base_pt*pow_2_[min_block_depth_width_] + pt;
                    //std::cout << "global pt = " << global_pt << ", ";
                    if(isoc_(global_pt)) {
                        is_occupied = true;
                    }
                }
                //std::cout << ", is occupied = " << is_occupied << std::endl;
                shrink_map_[i] = is_occupied;
            }

            shrink_isoc_ = [&](const Pointi<N> & pt) -> bool {
                if(isOutOfBoundary(pt, shrink_dim_)) { return true; }
                return shrink_map_[PointiToId(pt, shrink_dim_)];
            };

            // NOTICE: in override class's constructor, set all internal occ map state to occupied
            auto is_occupied_temp = [&](const Pointi<N> & pt) -> bool {
                if(isOutOfBoundary(pt, dim_)) { return true; }
                return raw_map_[PointiToId<N>(pt, dim_)];
            };
            isoc_dynamic_ = is_occupied_temp;

            sbt_ptr_ = std::make_shared<SpaceBinaryTreeAnyDimensionRaw<N> >(shrink_isoc_, shrink_dim_, 0, min_block_depth_width_);
            sbt_ptr_->initialize();
        }

        virtual const BlockWithTreePtr<N>& getInternalBlockPtr(const Pointi<N>& pt) const {
            if(isOutOfBoundary(pt, this->dim_)) { return nullptr; }
            return sbt_ptr_->getInternalBlockPtr(pt/pow_2_[min_block_depth_width_]);
        }

        float getObstacleDensity() const {
            // count passable children grid
            float count = 0;
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            for(int i=0; i<total_index; i++) {
                Pointi<N> pt = IdToPointi<N>(i, dim_);
                if(!isoc_dynamic_(pt)) {
                    count ++;
                }
            }
            return count / getTotalIndexOfSpace<N>(dim_);
        }

        void globalRecursiveUpdate() {
            sbt_ptr_->globalRecursiveUpdate();
        }

        ~ SpaceBinaryTreeShrink() {
            delete shrink_dim_;
            delete local_dim_;
        }

        bool getInternalOccState(const Pointi <N> &pt) const {
            return isoc_dynamic_(pt);
        }

        void setOccupiedState(const Pointi<N>& pt, const bool& is_occupied) {
//            std::cout << "set pt " << pt << " to " << is_occupied << std::endl;
            raw_map_[PointiToId<N>(pt, dim_)] = is_occupied;
            // check whether shrink map updated
            Pointi<N> base_pt, base_pt_global;
            for(int i=0; i<N; i++) {
                base_pt[i]        = pt[i] / pow_2_[min_block_depth_width_];
                base_pt_global[i] = base_pt[i] * pow_2_[min_block_depth_width_];
            }
//            std::cout << "base_pt = " << base_pt << std::endl;
//            std::cout << "base_pt_global = " << base_pt_global << std::endl;
//            std::cout << "local_total_index_ = " << local_total_index_ << std::endl;
//            std::cout << "local_dim_ = " << printDimInfo<N>(local_dim_) << std::endl;
            bool is_occupied_shrink = false;
            for(int j=0; j<local_total_index_; j++) {
                Pointi<N> pt = IdToPointi<N>(j, local_dim_);
                Pointi<N> global_pt = base_pt_global + pt;
                //std::cout << "pt = " << pt << ", base_pt_global + pt = " << base_pt_global + pt << std::endl;
                if(isOutOfBoundary(global_pt, dim_)) {
                    is_occupied_shrink = true;
                    break;
                }
                if(raw_map_[PointiToId<N>(global_pt, dim_)]) {
                    is_occupied_shrink = true;
                    break;
                }
            }
            Id shrink_id = PointiToId<N>(base_pt, shrink_dim_);
            //std::cout << "is_occupied_shrink = " << is_occupied_shrink << ", shrink_map_[shrink_id] = " << shrink_map_[shrink_id] << std::endl;

            if(shrink_map_[shrink_id] != is_occupied_shrink) {
                shrink_map_[shrink_id] = is_occupied_shrink;
                //std::cout << " call sbt_ptr_->setOccupiedState " << std::endl;
                sbt_ptr_->setOccupiedState(base_pt, is_occupied_shrink, true);
            }
            if(!is_occupied_shrink) {
                assert(sbt_ptr_->getInternalBlockPtr(base_pt) != nullptr);
            }
        }

        bool lineCrossObstacleRaw(const Pointi<N>& pt1, const Pointi<N>& pt2, IS_OCCUPIED_FUNC<N> is_occupied) {
            if(pt1 == pt2) return true;
            Line<N> line(pt1, pt2);
            int check_step = line.step;
            Pointi<N> pt;
            //std::cout << __FUNCTION__ << std::endl;
            for(int i=1; i<check_step; i++) {
                pt = line.GetPoint(i);
                raw_visited_pt_count_ ++;
                //std::cout << "raw_visited_pt_count_ = " << raw_visited_pt_count_ << std::endl;
                if(is_occupied(pt)) {
                    return true;
                }
            }
            return false;
        }


//        bool lineCrossObstacleSBT(const Pointi<N>& pt1, const Pointi<N>& pt2, const IS_OCCUPIED_FUNC<N>& is_occupied,
//                                  Pointis<N>& visited_pt,
//                                  int& count_of_block
//        ) {
//            //if(isOutOfBoundary(pt1, dim_) || isOutOfBoundary(pt2, dim_)) { return true; }
//            if(pt1 == pt2) return true;
//            //visited_pt.clear();
//            //count_of_block = 0;
//            Line<N> line(pt1, pt2);
//            int check_step = line.step;
//            Pointi<N> pt;
//            Id id;
//            int jump_step = 0;
//            for(int i=1; i<check_step; i++) {
//                pt = line.GetPoint(i);
//                SBT_visited_pt_count_ ++;
//                if(is_occupied(pt)) { return true; }
//                const auto& block_ptr = getInternalBlockPtr(pt);
//                // if in block, jump over current block
//                if(block_ptr != nullptr) {
//                    jump_step = findExitPointOfBlock(line, pt, i, static_cast<BlockPtr<N>>(block_ptr));
//                    //std::cout << " jump step " << jump_step << std::endl;
//                    i = i + jump_step;
//                    count_of_block ++;
//                }
//            }
//            return false;
//        }

        bool lineCrossObstacleSBT(const Pointi<N>& pt1, const Pointi<N>& pt2, const IS_OCCUPIED_FUNC<N>& is_occupied
                //,Pointis<N>& visited_pt
                , int& count_of_block
        ) {
            //if(isOutOfBoundary(pt1, dim_) || isOutOfBoundary(pt2, dim_)) { return true; }
            if(pt1 == pt2) { return is_occupied(pt1); }
            //visited_pt.clear();
            //count_of_block = 0;
            Line<N> line(pt1, pt2);
            int check_step = line.step;
            Pointi<N> pt;
            Id id;
            int jump_step = 0;
            //std::cout << __FUNCTION__ << std::endl;
            for(int i=1; i<check_step; i++) {
                pt = line.GetPoint(i);
                SBT_visited_pt_count_ ++;
                //std::cout << "SBT_visited_pt_count_ = " << SBT_visited_pt_count_ << std::endl;
                if(is_occupied(pt)) { return true; }
                //Pointi<N> pt_shrink = pt/pow_2_[min_block_depth_width_];
//                std::cout << "pt = " << pt << std::endl;
                const auto& block_ptr = getInternalBlockPtr(pt);


                // if in block, jump over current block
                if(block_ptr != nullptr) {
//                    std::cout << "block ptr = " << block_ptr->tree_node_->base_pt_ << std::endl;
//                    std::cout << "block_ptr->merged_block_id_ = " << block_ptr->merged_block_id_ << std::endl;
//                    std::cout << "block_ptr->depth_ = " << block_ptr->tree_node_->depth_ << std::endl;
//
//                    std::cout << "block_ptr->merged_block_id_ = " << block_ptr->merged_block_id_ << std::endl;
                    assert(block_ptr->merged_block_id_ != -1);
#if 0
                    jump_step = findExitPointOfBlock(line, pt, i, static_cast<BlockPtr<N>>(block_ptr));
#else
                    const auto& merged_block_ptr = sbt_ptr_->merged_block_ptrs_[block_ptr->merged_block_id_];
                    jump_step = findExitPointOfBlock(line, pt, i, merged_block_ptr->min_pt_ex_, merged_block_ptr->max_pt_ex_);
#endif
                    //std::cout << " jump step " << jump_step << std::endl;
//                    i = i + std::max(1, jump_step-1);
                    i = i + jump_step;
                    count_of_block ++;
                }
            }
            return false;
        }


        MergedBlockPtr<N> getInternalMergedBlockPtr(const Pointi<N>& pt) {
//            std::cout << "min_block_depth_width = " << min_block_depth_width_ << std::endl;
//            std::cout << "sbt pt = " << pt/pow_2_[min_block_depth_width_] << std::endl;
            return sbt_ptr_->getMergedBlockPtr(pt/pow_2_[min_block_depth_width_]);
        }

        // update massive pt together is more efficient
        void setNewOccAndPassablePts(const Pointis<N>& new_passable_pts, const Pointis<N>& new_occ_pts) {
            // only update changed node
            std::set<Id> updated_shrink_pt_id;

            for(const auto& pt : new_passable_pts) {
                raw_map_[PointiToId<N>(pt, dim_)] = false;
                // check whether shrink map updated
                Pointi<N> base_pt;
                for (int i = 0; i < N; i++) {
                    base_pt[i] = pt[i] / pow_2_[min_block_depth_width_];
                }
                updated_shrink_pt_id.insert(PointiToId<N>(base_pt, sbt_ptr_->dim_));
            }
            for(const auto& pt : new_occ_pts) {
                raw_map_[PointiToId<N>(pt, dim_)] = true;
                // check whether shrink map updated
                Pointi<N> base_pt;
                for (int i = 0; i < N; i++) {
                    base_pt[i] = pt[i] / pow_2_[min_block_depth_width_];
                }
                updated_shrink_pt_id.insert(PointiToId<N>(base_pt, sbt_ptr_->dim_));
            }
            Pointi<N> base_pt_shrink, base_pt_global;
            for(const auto& shrink_pt_id : updated_shrink_pt_id) {
                base_pt_shrink = IdToPointi<N>(shrink_pt_id, sbt_ptr_->dim_);
                base_pt_global = base_pt_shrink * pow_2_[min_block_depth_width_];

                bool is_occupied_shrink = false;
                for(int j=0; j<local_total_index_; j++) {
                    Pointi<N> pt = IdToPointi<N>(j, local_dim_);
                    Pointi<N> global_pt = base_pt_global + pt;
                    //std::cout << "pt = " << pt << ", base_pt_global + pt = " << base_pt_global + pt << std::endl;
                    if(isOutOfBoundary(global_pt, dim_)) {
                        is_occupied_shrink = true;
                        break;
                    }
                    if(raw_map_[PointiToId<N>(global_pt, dim_)]) {
                        is_occupied_shrink = true;
                        break;
                    }
                }
                if(shrink_map_[shrink_pt_id] != is_occupied_shrink) {
                    shrink_map_[shrink_pt_id] = is_occupied_shrink;
                    //std::cout << " call sbt_ptr_->setOccupiedState " << std::endl;
                    sbt_ptr_->setOccupiedState(base_pt_shrink, is_occupied_shrink, true);
                }
                if(!is_occupied_shrink) {
                    assert(sbt_ptr_->getInternalBlockPtr(base_pt_shrink) != nullptr);
                }
            }
            globalRecursiveUpdate();
        }

        IS_OCCUPIED_FUNC<N> isoc_;

        IS_OCCUPIED_FUNC<N> shrink_isoc_;

        DimensionLength* dim_;
        DimensionLength* shrink_dim_;

        DimensionLength* local_dim_;
        int local_total_index_;

        std::vector<bool> raw_map_;

        std::vector<bool> shrink_map_;

        std::vector<int> pow_2_; // precomputation of pow(2, x)

        int max_depth_ = 0;

        int min_block_depth_width_ = 0;

        std::shared_ptr<SpaceBinaryTreeAnyDimensionRaw<N> > sbt_ptr_;

        int raw_visited_pt_count_ = 0, SBT_visited_pt_count_ = 0;// for debug

        IS_OCCUPIED_FUNC<N> isoc_dynamic_; // notice, set state will change it

    };

    template<Dimension N>
    using SpaceBinaryTreeShrinkPtr = std::shared_ptr<SpaceBinaryTreeShrink<N> >;

}

#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_LAYERED_H
