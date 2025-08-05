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

            sbt_ptr_ = std::make_shared<SpaceBinaryTreeAnyDimensionRaw<N> >(shrink_isoc_, shrink_dim_, 0, min_block_depth_width_);
            sbt_ptr_->initialize();
        }

        void globalRecursiveUpdate() {
            sbt_ptr_->globalRecursiveUpdate();
        }

        ~ SpaceBinaryTreeShrink() {
            delete shrink_dim_;
            delete local_dim_;
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

        MergedBlockPtr<N> getInternalMergedBlockPtr(const Pointi<N>& pt) {
//            std::cout << "min_block_depth_width = " << min_block_depth_width_ << std::endl;
//            std::cout << "sbt pt = " << pt/pow_2_[min_block_depth_width_] << std::endl;
            return sbt_ptr_->getMergedBlockPtr(pt/pow_2_[min_block_depth_width_]);
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

    };

}

#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_LAYERED_H
