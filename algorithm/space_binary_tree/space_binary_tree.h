//
// Created by yaozhuo on 7/22/25.
//

#ifndef JUMPOVERBLOCK_SPACE_BINARY_TREE_H
#define JUMPOVERBLOCK_SPACE_BINARY_TREE_H

#include "common.h"

namespace freeNav::JOB {

    template<Dimension N>
    class SpaceBinaryTree {
    public:

        SpaceBinaryTree(const IS_OCCUPIED_FUNC <N> &isoc, DimensionLength *dim, int min_block_depth_width = 2)
                : isoc_(isoc), dim_(dim), min_block_depth_width_(min_block_depth_width) {

            std::cout << "min_block_depth_width = " << min_block_depth_width_ << std::endl;

            // initialize
            root_ = std::make_shared<TreeNode<N> >();
            root_->base_pt_ = Pointi<N>();
            // get max dimension length
            DimensionLength max_dim = 0;
            for (int i = 0; i < N; i++) {
                max_dim = std::max(max_dim, dim_[i]);
            }
            std::cout << "max_dim_length = " << max_dim << std::endl;
            max_depth_ = 1;
            while (true) {
                if (pow(2, max_depth_) < max_dim) {
                    max_depth_++;
                } else {
                    break;
                }
            }
            std::cout << "max_depth = " << max_depth_ << std::endl;
            // precomputation of pow(2, x)
            for (int dp = 0; dp <= std::max(max_depth_, (int) N); dp++) {
                pow_2_.push_back(pow(2, dp));
            }
            // precomputation of flag points
            flag_pts_ = GetFloorOrCeilFlag<N>();
            assert(flag_pts_.size() == pow_2_[N]);

            int size_of_vec = 1;
            level_offset_ = {0};
            for(int d=1; d<=max_depth_; d++) {
                size_of_vec = size_of_vec + pow_2_[d];
                level_offset_.push_back(level_offset_.back() + pow_2_[d-1]);
            }

            std::cout << " level_offset_ = ";
            for(int d=0; d<=max_depth_; d++) {
                std::cout << level_offset_[d] << " ";
            }
            std::cout << std::endl;

            tree_ptr_vec_.resize(size_of_vec, nullptr);

            // NOTICE: in override class's constructor, set all internal occ map state to occupied
        }

        int PointIdToTreeVecId(const Id id, int level) const {
            return level_offset_[level] + id / pow_2_[max_depth_-level];
        }

        void initialize() {
            // initialize max depth level tree node
            Pointi<N> pt;
            int max_width = pow_2_[max_depth_];
            DimensionLength* dim_max;
            for(int i=0; i<N; i++) {
                dim_max[i] = max_width;
            }
            int max_id = getTotalIndexOfSpace<N>(dim_max);
            for(int id=0; id<max_id; id++) {
                pt = IdToPointi<N>(id, dim_max);
                auto new_tree_node = std::make_shared<TreeNode<N> >(nullptr);
                new_tree_node->base_pt_ = pt;
                new_tree_node->occ_ = isoc_(pt);
                new_tree_node->mixed_state_ = false;
            }
            // from deep to top, construct parent node
            int child_id_global;
            for(int cur_level = max_depth_-1; cur_level>=0; cur_level--) {
                int cur_level_vec_offset = level_offset_[cur_level];
                int next_level_vec_offset = level_offset_[cur_level+1];
                for(int id=0; id<pow_2_[cur_level]; id++) {
                    int cur_tree_node_vec_id = cur_level_vec_offset + id;
                    bool all_the_same = true;
                    int is_occ = -1;
                    for(int child_id = id*pow_2_[N]; child_id < (id+1)*pow_2_[N]; child_id ++) {
                        child_id_global = next_level_vec_offset + child_id;
                        assert(tree_ptr_vec_[child_id_global] != nullptr);
                        if(tree_ptr_vec_[child_id_global]->mixed_state_) {
                            all_the_same = false;
                            break;
                        }
                        if(is_occ == -1) {
                            is_occ = tree_ptr_vec_[child_id_global]->occ_;
                        } else {
                            if(is_occ != tree_ptr_vec_[child_id_global]->occ_) {
                                all_the_same = false;
                                break;
                            }
                        }
                    }

                    auto new_tree_node = std::make_shared<TreeNode<N> >(nullptr);
                    new_tree_node->base_pt_ = id*pow_2_[max_depth_-cur_level]; // Pointi<N>
                    new_tree_node->occ_ = isoc_(pt);
                    new_tree_node->mixed_state_ = all_the_same;

                    if(all_the_same) {
                        for(int child_id = id*pow_2_[N]; child_id < (id+1)*pow_2_[N]; child_id ++) {
                            child_id_global = next_level_vec_offset + child_id;
                            assert(tree_ptr_vec_[child_id_global] != nullptr);
                            tree_ptr_vec_[child_id_global] = nullptr;
                        }
                    } else {
                        for(int child_id = id*pow_2_[N]; child_id < (id+1)*pow_2_[N]; child_id ++) {
                            child_id_global = next_level_vec_offset + child_id;
                            assert(tree_ptr_vec_[child_id_global] != nullptr);
                            tree_ptr_vec_[child_id_global]->pa_ = new_tree_node;
                            new_tree_node->children_.push_back(tree_ptr_vec_[child_id_global]);
                        }
                    }

                    tree_ptr_vec_[cur_level_vec_offset + id] = new_tree_node;

                    if(cur_level == 0) {
                        root_ = new_tree_node;
                    }
                }
            }

            initBlockPtrMap();
        }

        virtual void setInternalOccState(const Pointi<N>& pt, bool occ_state) = 0;

        virtual bool getInternalOccState(const Pointi<N>& pt) const = 0;

        virtual void setInternalBlockPtr(const Pointi<N>& pt, const BlockPtrRaw<N>& block_ptr) = 0;

        virtual const BlockPtrRaw<N>& getInternalBlockPtr(const Pointi<N>& pt) const = 0;

        virtual void clearInternalBlockPtr() = 0;

        void initBlockPtrMap() {
            std::cout << "start " << __FUNCTION__ << std::endl;
            clearInternalBlockPtr();

            std::vector<TreeNodePtr<N> > free_leaf_nodes = getAllPassableLeafNodes();
            for(const auto& leaf_node : free_leaf_nodes) {
                if(leaf_node->depth_ >= max_depth_ - min_block_depth_width_) { continue; } // limit minimum size of blocks
//                BlockPtrRaw<N> block_ptr = new Block<N>();
                BlockPtrRaw<N> block_ptr = std::make_shared<Block<N> >();
                block_ptr->min_ = leaf_node->base_pt_;
                Pointi<N> offset; offset.setAll(pow_2_[max_depth_-leaf_node->depth_]-1);
                block_ptr->max_ = leaf_node->base_pt_ + offset;
                //leaf_node->block_ptr_ = block_ptr;
                setBlockPtrForNode(leaf_node, block_ptr);
            }
            std::cout << "finish " << __FUNCTION__ << std::endl;
        }

        // set all grid in current node range to the same block_ptr
        void setBlockPtrForNode(const TreeNodePtr<N>& node, const BlockPtrRaw<N>& block_ptr) {
            if(block_ptr != nullptr &&
               (isOutOfBoundary(block_ptr->min_, dim_) || isOutOfBoundary(block_ptr->max_, dim_))) {
                return;
            }
            Pointi<N> offset; offset.setAll(pow_2_[max_depth_-node->depth_]-1);
            DimensionLength local_dim[N];
            for(int d=0; d<N; d++) { local_dim[d] = offset[d]+1; }
            Id local_total_index = getTotalIndexOfSpace<N>(local_dim), global_id;

            Pointi<N> local_pt, global_pt;
            for(Id id=0; id<local_total_index; id++) {
                local_pt = IdToPointi<N>(id, local_dim);
                global_pt = node->base_pt_ + local_pt;
                if(block_ptr != nullptr && isOutOfBoundary(global_pt, dim_)) {
                    //continue; // if larger than block_ptr_map_, it is out of map
                    //block ptr shouldn't out of map
                    assert(0);
                }
                if(isOutOfBoundary(global_pt, dim_)) {
                    continue;
                }
                //global_id = PointiToId(global_pt, dim_);
                //block_ptr_map_[global_id] = block_ptr;
                setInternalBlockPtr(global_pt, block_ptr);
            }
        }

        TreeNodePtrs<N> getAllPassableLeafNodes() const {
            //std::cout << "-- " << __FUNCTION__ << std::endl;
            TreeNodePtrs<N> nodes = { root_ }, next_nodes, retv;
            int dp = 0;
            while (!nodes.empty()) {
                //std::cout << " depth = " << dp << ": " << std::endl;
                for(int i=0; i<nodes.size(); i++) {
                    assert(nodes[i]->depth_ == dp);
                    assert(nodes[i]->children_.size() == pow_2_[N]);
                    if(!nodes[i]->mixed_state_ && !nodes[i]->occ_) {
                        retv.push_back(nodes[i]);
                    }
                    for(int j=0; j<pow_2_[N]; j++) {
                        if(nodes[i]->children_[j] != nullptr) {
                            next_nodes.push_back(nodes[i]->children_[j]);
                        }
                    }
                    //std::cout << std::endl;
                }
                nodes.clear();
                std::swap(nodes, next_nodes);
                dp ++;
            }
            return retv;
        }

        IS_OCCUPIED_FUNC <N> isoc_; // notice, setOccupiedState will not change it,
        // so it is wrong after call setOccupiedState after initialize

        DimensionLength *dim_;

        TreeNodePtr <N> root_ = nullptr;

        int max_depth_ = 0;

        int min_block_depth_width_ = 1; // the minimum block width is pow(2, min_block_depth_width_)

        std::vector<int> pow_2_; // precomputation of pow(2, x)

        Pointis <N> flag_pts_; // precomputation of all flag points

        bool initialized_ = false; // enable update block ptr only after initialized

        TreeNodePtrs<N> tree_ptr_vec_; // size = 1 + pow(2, N) + pow(2, 2N) +...

        std::vector<int> level_offset_; // offset to visit level N's tree node in tree_ptr_vec_

    };

}
#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_H
