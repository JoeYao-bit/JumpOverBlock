//
// Created by yaozhuo on 2025/4/24.
//

#ifndef JUMPOVERBLOCK_SPACE_BINARY_TREE_RAW_H
#define JUMPOVERBLOCK_SPACE_BINARY_TREE_RAW_H

#include "../line_of_sight_jump_between_block.h"
#include "common.h"
#include "freeNav-base/basic_elements/misc.h"
namespace freeNav::JOB {

    // store all passable block
    // if a block (tree node)'s all children node is nullptr, it is passable
    // otherwise, some part of it are passable and others are occupied,
    // those passable are non-null, unpassable are null
    template<Dimension N>
    class SpaceBinaryTreeRaw {
    public:

        SpaceBinaryTreeRaw(const IS_OCCUPIED_FUNC<N>& isoc, DimensionLength* dim,
                           int min_block_depth_width = 2,
                           int external_min_block_depth_width = 0)
        : isoc_(isoc), dim_(dim), min_block_depth_width_(min_block_depth_width),
          external_min_block_depth_width_(external_min_block_depth_width) {
            // initialize
            root_ = new TreeNode<N>();
            root_->base_pt_ = Pointi<N>();
            // get max dimension length
            DimensionLength max_dim = 0;
            for(int i=0; i<N; i++) {
                max_dim = std::max(max_dim, dim_[i]);
            }
            all_1_pt_.setAll(1);
            //std::cout << "max_dim_length = " << max_dim << std::endl;
            max_depth_ = 1;
            while(true) {
                if(pow(2, max_depth_) < max_dim) {
                    max_depth_ ++;
                } else {
                    break;
                }
            }
            //std::cout << "max_depth = " << max_depth_ << std::endl;
            // precomputation of pow(2, x)
            for(int dp=0; dp<=std::max(max_depth_, (int)N); dp++) {
                pow_2_.push_back(pow(2, dp));
            }
            all_external_offset_.setAll(pow_2_[external_min_block_depth_width_] - 1);
            // precomputation of flag points
            flag_pts_ = GetFloorOrCeilFlag<N>();
            //assert(flag_pts_.size() == pow_2_[N]);

            //std::cout << "min_block_depth_width = " << min_block_depth_width_ << std::endl;

            // NOTICE: in override class's constructor, set all internal occ map state to occupied
            auto is_occupied_temp = [&](const Pointi<N> & pt) -> bool {
                return getInternalOccState(pt);
            };
            isoc_dynamic_ = is_occupied_temp;
        }

        void releaseLeafNodes() {
//            //std::cout << "-- " << __FUNCTION__ << std::endl;
//            TreeNodePtrs<N> nodes = { root_ }, next_nodes, retv;
//            int dp = 0;
//            while (!nodes.empty()) {
//                //std::cout << " depth = " << dp << ": " << std::endl;
//                for(int i=0; i<nodes.size(); i++) {
//                    assert(nodes[i]->depth_ == dp);
//                    assert(nodes[i]->children_.size() == pow_2_[N]);
//                    if(!nodes[i]->mixed_state_ && !nodes[i]->occ_) {
//                        if(nodes[i]->block_ptr_ != nullptr) {
//                            delete nodes[i]->block_ptr_;
//                            nodes[i]->block_ptr_ = nullptr;
//                        }
//                    } else {
//                        assert(nodes[i]->block_ptr_ == nullptr);
//                    }
//                    for(int j=0; j<pow_2_[N]; j++) {
//                        if(nodes[i]->children_[j] != nullptr) {
//                            next_nodes.push_back(nodes[i]->children_[j]);
//                        }
//                    }
//                    //std::cout << std::endl;
//                }
//                nodes.clear();
//                std::swap(nodes, next_nodes);
//                dp ++;
//            }
            delete root_;
            root_ = nullptr;
        }

        ~SpaceBinaryTreeRaw () {
            releaseLeafNodes();
        }

        void initBlockPtrMap() {
            //std::cout << "start " << __FUNCTION__ << std::endl;
            clearInternalBlockPtr();

            std::vector<TreeNodePtr<N> > free_leaf_nodes = getAllPassableLeafNodes();
            for(const auto& leaf_node : free_leaf_nodes) {
                if(leaf_node->depth_ > max_depth_ - min_block_depth_width_) { continue; } // limit minimum size of blocks
                BlockWithTreePtr<N> block_ptr = std::make_shared<BlockWithTree<N> >();
                block_ptr->min_ = leaf_node->base_pt_;
                Pointi<N> offset; offset.setAll(pow_2_[max_depth_-leaf_node->depth_]-1);
                block_ptr->max_ = leaf_node->base_pt_ + offset;

                block_ptr->tree_node_ = leaf_node;

                //leaf_node->block_ptr_ = block_ptr;
                setBlockPtrForNode(leaf_node, block_ptr);
            }
            //std::cout << "finish " << __FUNCTION__ << std::endl;
        }

        // need call this after construction
        virtual void initialize() {
            MSTimer mst; 
            std::cout << getTimeInYMD<2>() << " start initialize of SBT of space dim " << printDimInfo<N>(dim_) << std::endl;
            // initialize of space
            Id total_index = getTotalIndexOfSpace<N>(dim_);

            initialized_ = false;
            root_->occ_ = true;
            for(Id id=0; id<total_index; id++) {
                Pointi<N> pt = IdToPointi<N>(id, dim_);
                if(!isoc_(pt)) {
                    setOccupiedState(pt, false, false);
                    //occ_map_[id] = false;
                    setInternalOccState(pt, false);
                }
            }

            // initialize of block_ptr_map_
            initBlockPtrMap();
            initialized_ = true;
            std::cout << getTimeInYMD<2>() << " finish initialize of SBT of space dim " << printDimInfo<N>(dim_) << " in " << mst.elapsed()/1e3 << "s" << std::endl;
            mst.reset();
            if(external_min_block_depth_width_ != 0) {
                // merge blocks is time consuming in large map, so do not use it in raw map, only use it in shrink map
                mergePassableBlocksViaDecisionTree();
            //std::cout << getTimeInYMD<2>() << " finish merge blocks of SBT of space dim " << printDimInfo<N>(dim_) << " in " << mst.elapsed()/1e3 << "s" << std::endl;
            }

        }



        virtual void setInternalOccState(const Pointi<N>& pt, bool occ_state) = 0;

        virtual bool getInternalOccState(const Pointi<N>& pt) const = 0;

        virtual void setInternalBlockPtr(const Pointi<N>& pt, const BlockWithTreePtr<N>& block_ptr) = 0;

        virtual const BlockWithTreePtr<N>& getInternalBlockPtr(const Pointi<N>& pt) const = 0;

        virtual void clearInternalBlockPtr() = 0;

        // set all grid in current node range to the same block_ptr
        void setBlockPtrForNode(const TreeNodePtr<N>& node, const BlockWithTreePtr<N>& block_ptr) {
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

        // update state of node, do not update isoc
        // set passable to unpassable may result new tree node and erase existing node
        // and update block_ptr_map_
        void setOccupiedState(const Pointi<N>& pt, bool is_occupied, bool update_block = true) {
            if(isOutOfBoundary(pt, dim_)) { return ; }

            //Id id = PointiToId(pt, dim_);
            //occ_map_[id] = is_occupied;

            setInternalOccState(pt, is_occupied);

            TreeNodePtr<N> buffer = root_;
            for(int dp=0; dp <= max_depth_; dp++) {
                //std::cout << "buffer->depth_ = " << buffer->depth_ << std::endl;
                if(!buffer->mixed_state_) {
                    // if reach a leaf node
//                    if(buffer->block_ptr_ != nullptr) {
//                        delete buffer->block_ptr_;
//                        buffer->block_ptr_ = nullptr;
//                    }
                    if(buffer->occ_ == is_occupied) {
                        // if current block is already the same state, do nothing
//                        std::cout << "leaf node has the same state, do nothing" << std::endl;
                        return;
                    }
                    // reach a leaf node, and its not the same state
                    //std::cout << "reach a leaf node, and its not the same state" << std::endl;
                    buffer->mixed_state_ = true;
                    // update block_ptr_map_
                    if(is_occupied && initialized_ && update_block) {
                        // set all block ptr in current leaf node as nullptr
                        setBlockPtrForNode(buffer, nullptr);
                    }
                    for(; dp<max_depth_; dp++) {
                        size_t index = getIndex(pt, dp);
                        for (int i = 0; i < pow_2_[N]; i++) {
                            buffer->children_[i] = new TreeNode<N>(buffer);//std::make_shared<TreeNode<N> >(buffer);
                            int zoom_ratio = pow_2_[max_depth_ - dp - 1];
                            buffer->children_[i]->base_pt_ = buffer->base_pt_ + flag_pts_[i].multi(zoom_ratio);
                            buffer->children_[i]->occ_ = !is_occupied;
                            buffer->children_[i]->mixed_state_ = false;
                            // set block ptr when is_occupied = true, as this action may create multiple small blocks
                            // as when is_occupied = true, no block ptr will be set in recurAndUpdate
                            if (is_occupied && i != index &&
                                (buffer->children_[i]->depth_ <= max_depth_ - min_block_depth_width_) &&
                                update_block) {
                                BlockWithTreePtr<N> block_ptr = std::make_shared<BlockWithTree<N> >();
                                block_ptr->min_ = buffer->children_[i]->base_pt_;
                                int space_width = pow_2_[max_depth_ - buffer->children_[i]->depth_];
                                Pointi<N> offset;
                                offset.setAll(space_width - 1);
                                block_ptr->max_ = buffer->children_[i]->base_pt_ + offset;
                                block_ptr->tree_node_ = buffer->children_[i];
                                //std::cout << "create block min/max " << block_ptr->min_ << ", " << block_ptr->max_ << std::endl;
                                setBlockPtrForNode(buffer->children_[i], block_ptr);
                                //buffer->children_[i]->block_ptr_ = block_ptr;
                            }
                        }
                        buffer = buffer->children_[index];
                        buffer->mixed_state_ = true;
                        //                        if(buffer->block_ptr_ != nullptr) {
                        //                            delete buffer->block_ptr_;
                        //                            buffer->block_ptr_ = nullptr;
                        //                        }
                    }
                    buffer->occ_ = is_occupied;
                    buffer->mixed_state_ = false; // last node is leaf node, it is not mixed state
                    if(!is_occupied && min_block_depth_width_ == 0) {
                        BlockWithTreePtr<N> block_ptr = std::make_shared<BlockWithTree<N> >();
                        block_ptr->min_ = buffer->base_pt_;
                        block_ptr->max_ = buffer->base_pt_;
                        block_ptr->tree_node_ = buffer;
                        setInternalBlockPtr(buffer->base_pt_, block_ptr);
                    }
                    break;
                } else {
                    size_t index = getIndex(pt, dp);
                    buffer = buffer->children_[index];
                }
            }
            //printTree();
            // if a node's all children node is passable, set all it's children node to nullptr
            TreeNodePtr<N> parent = buffer->parent_;
            recurAndUpdate(parent, update_block);
        }

        // set mixed_state to false and all children to nullptr if all child are occupied or unpassable
        // and update block_ptr_map_
        void recurAndUpdate(TreeNodePtr<N> parent, bool update_block = true) {
            //std::cout << "recurAndUpdate" << std::endl;
            assert(parent->depth_ == max_depth_ - 1);
            // remove child node if all child are passable
            for(int dp=max_depth_-1; dp>=0; dp--) {
                assert(parent != nullptr);
                bool all_same_state = true;
                // check whether all child is the same state,
                // if is the same state, set all child to nullptr and mixed_state to false
                for(int i=0; i<pow_2_[N]; i++) {
                    if((parent->children_[0]->occ_ ^ parent->children_[i]->occ_)
                    || parent->children_[i]->mixed_state_) {
                        all_same_state = false;
                        break;
                    }
                }
                if(all_same_state) {
                    //std::cout << "detect all same state" << std::endl;
                    parent->occ_ = parent->children_[0]->occ_;
                    parent->mixed_state_ = false;
                    // if all the same state, remove all child node
                    for(int i=0; i<pow_2_[N]; i++) {
//                        if(parent->children_[i]->block_ptr_ != nullptr) {
//                            delete parent->children_[i]->block_ptr_;
//                            parent->children_[i]->block_ptr_ = nullptr;
//                        }
                        if(parent->children_[i] != nullptr) {
                            delete parent->children_[i];
                            parent->children_[i] = nullptr;
                        }
                    }
                    if(initialized_) {
                        // if set to passable, check whether create big block
                        // limit minimum size of blocks
                        //std::cout << "create block, parent->occ_ = " << parent->occ_ << std::endl;
                        if(!parent->occ_ && (parent->depth_ <= max_depth_ - min_block_depth_width_) && update_block) {
                            BlockWithTreePtr<N> block_ptr = std::make_shared<BlockWithTree<N> >();
                            block_ptr->min_ = parent->base_pt_;
                            int space_width = pow_2_[max_depth_-parent->depth_];
                            Pointi<N> offset; offset.setAll(space_width-1);
                            block_ptr->max_ = parent->base_pt_ + offset;
                            block_ptr->tree_node_ = parent;
                            //std::cout << "create block min/max " << block_ptr->min_ << ", " << block_ptr->max_ << std::endl;
                            setBlockPtrForNode(parent, block_ptr);
                            //parent->block_ptr_ = block_ptr;
                        }
                    }
                } else {
                    parent->mixed_state_ = true;
                }
                parent = parent->parent_;
            }
        }

        void setNewOccAndPassablePts(const Pointis<N>& new_passable_pts, const Pointis<N>& new_occ_pts) {
            // only update changed node
            for (const auto &new_free : new_passable_pts) {
                setOccupiedState(new_free, false, true);
            }
            for (const auto &new_occ : new_occ_pts) {
                setOccupiedState(new_occ, true, true);
            }
//            if (!update_block_ptr_realtime) {
//                initBlockPtrMap();
//            }
            // const auto& all_leaves = getAllPassableLeafNodes();
            // std::set<BlockWithTreePtr<N>, BlockCompare<N>> leave_merged_map;
            // for(const auto& leaf_node : all_leaves) {
            //     if(leaf_node->depth_ > max_depth_ - min_block_depth_width_) { continue; }
            //     //std::cout << "leaf node " << leaf_node << " dp = " << leaf_node->depth_ << ", base pt = " << leaf_node->base_pt_ << std::endl;
            //     const auto& block_ptr = getInternalBlockPtr(leaf_node->base_pt_);
            //     leave_merged_map.insert(block_ptr);
            // }
            // int total_index = getTotalIndexOfSpace<N>(dim_);
            // for(int id=0; id<total_index; id++) {
            //     Pointi<N> pt = IdToPointi<N>(id, dim_);
            //     auto block_ptr = getInternalBlockPtr(pt);
            //     if(block_ptr != nullptr) {
            //         if(leave_merged_map.find(block_ptr) == leave_merged_map.end()) {
            //             std::cout << "haha 0" << std::endl;
            //         }
            //         //assert(leave_merged_map.find(block_ptr) != leave_merged_map.end());
            //     }
            // }
            globalRecursiveUpdate();
        }

        // have no real use, just keep pace with SBT new
        void globalRecursiveUpdate() {
            if(external_min_block_depth_width_ != 0) {
                // merge blocks is time consuming in large map, so do not use it in raw map, only use it in shrink map
                mergePassableBlocksViaDecisionTree(); 
            }
        }

        TreeNodePtr<N> getLeafNode(const Pointi<N>& pt) const {
            TreeNodePtr<N> buffer = root_;
            for(int dp=0; dp<=max_depth_; dp++) {
                if(!buffer->mixed_state_) {
                    return buffer;
                }
                size_t index = getIndex(pt, dp);
                buffer = buffer->children_[index];
            }
            std::cout << "find no leaf node, shouldn't reach here" << std::endl;
            assert(0);
            return buffer;
        }

        // find the leaf node that contain current pt, and return its occ
        bool isOccupied(const Pointi<N>& pt) const {
            TreeNodePtr<N> buffer = root_;
            for(int dp=0; dp<=max_depth_; dp++) {
                if(!buffer->mixed_state_) {
                    return buffer->occ_;
                }
                size_t index = getIndex(pt, dp);
                buffer = buffer->children_[index];
            }
            std::cout << "find no leaf node, shouldn't reach here" << std::endl;
            assert(0);
            return true;
        }

        // do not use block_ptr_map_, efficient in memory space but more time consuming
//        bool lineCrossObstacle(const Pointi<N>& pt1, const Pointi<N>& pt2) const {
//            Line<N> line(pt1, pt2);
//            int check_step = line.step;
//            Pointi<N> pt;
//            int jump_step = 0;
//            for(int i=1; i<check_step; i++) {
//                pt = line.GetPoint(i);
//                TreeNodePtr<N> leaf_node = getLeafNode(pt);
//                if(leaf_node->occ_) {
//                    return true;
//                } else {
//                    // if come across a passable block, get its width and boundary,
//                    // construct a local BlockPtrRaw and jump
//                    BlockPtrRaw<N> block_ptr = std::make_shared<Block<N> >();
//                    block_ptr->min_ = leaf_node->base_pt_;
//                    Pointi<N> offset; offset.setAll(pow_2_[max_depth_-leaf_node->depth_]-1);
//                    block_ptr->max_ = leaf_node->base_pt_ + offset;
//
//                    jump_step = findExitPointOfBlock(line, pt, i, block_ptr);
//                    //std::cout << " jump step " << jump_step << std::endl;
//                    i = i + jump_step;
//                }
//            }
//            return false;
//        }


        std::set<BlockWithTreePtr<N> >
        tryExpandBlockInDirection(int dim,
                                  const MergedBlockPtr<N>& block_node,
                                  const std::map<BlockWithTreePtr<N>, bool, BlockCompare<N> >& leave_merged_map) const {

            Pointi<N> dim_of_block = block_node->max_pt_ - block_node->min_pt_ + all_1_pt_;
            DimensionLength plain[N - 1];
            for (Dimension i = 0; i < dim / 2; i++) {
                plain[i] = dim_of_block[i];
            }
            for (Dimension i = dim / 2 + 1; i < N; i++) {
                plain[i - 1] = dim_of_block[i];
            }
            // get the dimension of the N-1 dimension plain
            Id total_plain_index = getTotalIndexOfSpace<N - 1>(plain);
            Pointi<N - 1> plain_pt;
            Pointi<N> new_plain_pt;
            std::vector<Id> new_plain_ids;
            // set the origin of plain
            Pointi<N> origin_of_plain = block_node->min_pt_;
            if (dim % 2 == 1) {
                origin_of_plain[dim / 2] = block_node->max_pt_[dim / 2] + 1;
            } else {
                origin_of_plain[dim / 2] -= 1;
            }
            // the block in the surface must form a plain that have exact the size of the plain
            //std::cout << "new plain total index = " << total_plain_index << std::endl;
            //std::cout << "new plain pt at direct " << dim << ": ";
            bool expandable = true;
            BlockWithTreePtr<N> neighbor_block_ptr = nullptr;
            int expected_num_of_neighbor_block = 0;
            std::set<BlockWithTreePtr<N> > neighbor_block_ptrs;
            for (Id id = 0; id < total_plain_index; id++) {
                plain_pt = IdToPointi<N - 1>(id, plain);
                new_plain_pt = origin_of_plain.addPlainOffset(plain_pt, dim);
                //std::cout << new_plain_pt << " " << std::endl;
                // if out of boundary, cannot expand
                if(isOutOfBoundary(new_plain_pt, dim_)) {
                    expandable = false;
                    //std::cout << "out of bound, expand failed " << std::endl;
                    break;
                }
                BlockWithTreePtr<N> temp_block_ptr = getInternalBlockPtr(new_plain_pt);
                if(temp_block_ptr == nullptr) {
                    expandable = false;
                    //std::cout << "not in block ptr, expand failed " << std::endl;
                    break;
                }
                const auto& temp_tree_ptr = temp_block_ptr->tree_node_;
                //std::cout << "temp_tree_ptr = " << temp_tree_ptr << ", dp = "
                //          << temp_tree_ptr->depth_ << ", base_pt = " << temp_tree_ptr->base_pt_ << std::endl;
                // if tree ptr is mixed state or occ is true, cannot expand
                if(temp_tree_ptr->mixed_state_ || temp_tree_ptr->occ_ == true || leave_merged_map.at(temp_block_ptr) == true) {
                    expandable = false;
                    //std::cout << "not in leaf node or have been expand, expand failed " << std::endl;
                    break;
                }
                // cannot exceed current block's range
                if(temp_block_ptr->max_ > block_node->max_pt_ || temp_block_ptr->min_ < block_node->min_pt_) {
                    expandable = false;
                    //std::cout << "neighbor area out of range, expand failed " << std::endl;
                    break;
                }
                if(neighbor_block_ptr == nullptr) {
                    neighbor_block_ptr = temp_block_ptr;
                    neighbor_block_ptrs.insert(neighbor_block_ptr);
                    int area_of_neighbor_node = pow(pow_2_[max_depth_-neighbor_block_ptr->tree_node_->depth_], N-1);
                    expected_num_of_neighbor_block = total_plain_index / area_of_neighbor_node;
                    neighbor_block_ptrs.insert(neighbor_block_ptr);
                } else {
                    if(neighbor_block_ptr->tree_node_->depth_ != temp_tree_ptr->depth_) {
                        //std::cout << "neighbor block have not same size , expand failed " << std::endl;
                        expandable = false;
                        break;
                    } else {
                        neighbor_block_ptrs.insert(temp_block_ptr);
                    }
                }
            }
            //std::cout << std::endl;
            if(!expandable) {
                return {};
            }
            //std::cout << "expected_num_of_neighbor_block = " << expected_num_of_neighbor_block << std::endl;
            //std::cout << "neighbor_block_ptrs.size() = " << neighbor_block_ptrs.size() << std::endl;
            if(expected_num_of_neighbor_block == neighbor_block_ptrs.size()) {
                return neighbor_block_ptrs;
            } else {
                //std::cout << "number of neighbor block not meet expected size , expand failed " << std::endl;
                return {};
            }
        }

        // merge block with neighbor block until local maximize
        void mergePassableBlocksViaDecisionTree() {
            USTimer mst;
            merged_block_ptrs_.clear();
            // big block start, small block end
            const auto& all_leaves = getAllPassableLeafNodes();
            std::map<BlockWithTreePtr<N>, bool, BlockCompare<N>> leave_merged_map;
            BlockWithTreePtrs<N> all_blocks;
            for(const auto& leaf_node : all_leaves) {
                if(leaf_node->depth_ > max_depth_ - min_block_depth_width_) { continue; }
                //std::cout << "leaf node " << leaf_node << " dp = " << leaf_node->depth_ << ", base pt = " << leaf_node->base_pt_ << std::endl;
                const auto& block_ptr = getInternalBlockPtr(leaf_node->base_pt_);
                leave_merged_map.insert({block_ptr, false});
                all_blocks.push_back(block_ptr);
            }
            //return ;
            //std::cout << "flag 1" << std::endl;
            for(const auto& block_ptr : all_blocks) {
                if(block_ptr->tree_node_->depth_ > max_depth_ - min_block_depth_width_) { continue; }
                //std::cout << "temp_leaf = " << temp_leaf << std::endl;
                if(leave_merged_map[block_ptr]) { continue; }
                // initial root block
                MergedBlockPtr<N> root = std::make_shared<MergedBlock<N> >();
                root->min_pt_ = block_ptr->min_;
                root->max_pt_ = block_ptr->max_;

                root->min_pt_ex_ = block_ptr->min_*pow_2_[external_min_block_depth_width_];
                root->max_pt_ex_ = block_ptr->max_*pow_2_[external_min_block_depth_width_] + all_external_offset_;

                //std::cout << "root block: min_pt_ = " << root->min_pt_ << " / max_pt = " << root->max_pt_  << std::endl;

                MergedBlockPtr<N> largest_merged_block_ptr = root;
                root->block_ptrs_ = {block_ptr};

                int largest_size = getTotalIndexOfSpace(largest_merged_block_ptr->max_pt_ - largest_merged_block_ptr->min_pt_);

                MergedBlockPtrs<N> buffer = {root}, next_buffer;

//                leave_merged_map[block_ptr] = true;
//                block_ptr->merged_block_id_ = merged_block_ptrs_.size();
                int depth_of_iter = 0;
                while(!buffer.empty()) {
                    next_buffer.clear();
                    //std::cout << "depth_of_iter = " << depth_of_iter << std::endl;
                    // too deep may need lots of memory, so limit the max depth
                    if(N > 2 && depth_of_iter > 12) { break; }
                    for(const auto& block_node : buffer) {
                        // expand current block until cannot expand
                        for (int dim = 0; dim < 2 * N; dim++) {
                            // if there are required number of tree node in this direction
                            // we say we found a legal expansion
                            auto neighbor_block_ptrs = tryExpandBlockInDirection(dim, block_node, leave_merged_map);
                            if(!neighbor_block_ptrs.empty()) {
                                //std::cout << "expand in dir " << dim << " success" << std::endl;
                                MergedBlockPtr<N> new_block_node = std::make_shared<MergedBlock<N> >();
                                new_block_node->min_pt_ = block_node->min_pt_;
                                new_block_node->max_pt_ = block_node->max_pt_;
                                //new_block_node->parent_ = block_node;
                                new_block_node->block_ptrs_ = block_node->block_ptrs_;
                                int expand_dist = pow_2_[max_depth_ - (*neighbor_block_ptrs.begin())->tree_node_->depth_];
                                // update block size
                                if(dim % 2 == 1) {
                                    new_block_node->max_pt_[dim/2] = new_block_node->max_pt_[dim/2] + expand_dist;
                                } else {
                                    new_block_node->min_pt_[dim/2] = new_block_node->min_pt_[dim/2] - expand_dist;
                                }
                                new_block_node->min_pt_ = new_block_node->min_pt_;
                                new_block_node->max_pt_ = new_block_node->max_pt_;

                                new_block_node->min_pt_ex_ = new_block_node->min_pt_*pow_2_[external_min_block_depth_width_];
                                new_block_node->max_pt_ex_ = new_block_node->max_pt_*pow_2_[external_min_block_depth_width_] + all_external_offset_;

                                for(const auto& neighbor_block_ptr : neighbor_block_ptrs) {
                                    new_block_node->block_ptrs_.push_back(neighbor_block_ptr);
                                }
                                //block_node->children_.push_back(new_block_node);
                                next_buffer.push_back(new_block_node);
                                // update largest merged block
                                int new_size = getTotalIndexOfSpace(new_block_node->max_pt_ - new_block_node->min_pt_);
                                if(largest_size < new_size) {
                                    largest_merged_block_ptr = new_block_node;
                                    largest_size = new_size;
                                }
                                //std::cout << "merged block from " << block_node->min_pt_ << "<->" << block_node->max_pt_ << " to "
                                //          << new_block_node->min_pt_ << "<->" << new_block_node->max_pt_ << std::endl;
                            } else {
                                //std::cout << "expand in dir " << dim << " failed" << std::endl;
                            }
                            //std::cout << std::endl;
                        }
                    }
                    depth_of_iter ++;
                    std::swap(buffer, next_buffer);
                }
                merged_block_ptrs_.push_back(largest_merged_block_ptr);
                for(const auto& neighbor_block_ptr : largest_merged_block_ptr->block_ptrs_) {
                    leave_merged_map[neighbor_block_ptr] = true;
                    neighbor_block_ptr->merged_block_id_ = merged_block_ptrs_.size() - 1;
                }
                // save the largest merged block
                //break; // debug only, means just try expand one block
            }
            //std::cout << "all merged block = ";
            //for(const auto& merged_block_ptr : merged_block_ptrs_) {
            //    std::cout << merged_block_ptr->min_pt_ << "<->" << merged_block_ptr->max_pt_ << ", ";
            //}
            //std::cout << std::endl;
            std::cout << __FUNCTION__ << " take " << mst.elapsed()/1e3 << " ms" << std::endl;

            // debug
            // for(const auto& leaf_node : all_leaves) {
            //     if(leaf_node->depth_ > max_depth_ - min_block_depth_width_) { continue; }
            //     //std::cout << "leaf node " << leaf_node << " dp = " << leaf_node->depth_ << ", base pt = " << leaf_node->base_pt_ << std::endl;
            //     const auto& block_ptr = getInternalBlockPtr(leaf_node->base_pt_);
            //     assert(leave_merged_map[block_ptr]);
            //     assert(block_ptr->merged_block_id_ >= 0);
            // }
            // int total_index = getTotalIndexOfSpace<N>(dim_);
            // for(int id=0; id<total_index; id++) {
            //     Pointi<N> pt = IdToPointi<N>(id, dim_);
            //     auto block_ptr = getInternalBlockPtr(pt);
            //     if(block_ptr != nullptr) {
            //         if(leave_merged_map.find(block_ptr) == leave_merged_map.end()) {
            //             std::cout << "haha" << std::endl;
            //         }
            //         assert(leave_merged_map.find(block_ptr) != leave_merged_map.end());
            //         assert(leave_merged_map[block_ptr]);
            //         assert(block_ptr->merged_block_id_ >= 0);
            //     }
            // }
        }


        int raw_visited_pt_count_ = 0, SBT_visited_pt_count_ = 0;// for debug

        bool lineCrossObstacleRaw(const Pointi<N>& pt1, const Pointi<N>& pt2, IS_OCCUPIED_FUNC<N> is_occupied) {
            if(pt1 == pt2) return is_occupied(pt1);
            Line<N> line(pt1, pt2);
            int check_step = line.step;
            Pointi<N> pt;
            for(int i=1; i<check_step; i++) {
                pt = line.GetPoint(i);
                raw_visited_pt_count_ ++;
                if(is_occupied(pt)) {
                    return true;
                }
            }
            return false;
        }

        bool lineCrossObstacleSBT(const Pointi<N>& pt1, const Pointi<N>& pt2, const IS_OCCUPIED_FUNC<N>& is_occupied,
                               //Pointis<N>& visited_pt,
                               int& count_of_block
                               ) {
            //if(isOutOfBoundary(pt1, dim_) || isOutOfBoundary(pt2, dim_)) { return true; }
            if(pt1 == pt2) return is_occupied(pt1);
            //visited_pt.clear();
            //count_of_block = 0;
            Line<N> line(pt1, pt2);
            int check_step = line.step;
            Pointi<N> pt;
            Id id;
            int jump_step = 0;
            for(int i=1; i<check_step; i++) {
                pt = line.GetPoint(i);
                SBT_visited_pt_count_ ++;

//                if(getInternalOccState(pt)) { return true; }

                if(is_occupied(pt)) { return true; }

                const auto& block_ptr = getInternalBlockPtr(pt);

                // if in block, jump over current block
                if(block_ptr != nullptr) {
                    jump_step = findExitPointOfBlock(line, pt, i, static_cast<BlockPtr<N>>(block_ptr));
                    //std::cout << " jump step " << jump_step << std::endl;
                    i = i + jump_step;
                    count_of_block ++;
                }
            }
            return false;
        }

        MergedBlockPtr<N> getMergedBlockPtr(const Pointi<N>& pt) {
            const auto& block_ptr = getInternalBlockPtr(pt);
            if(block_ptr == nullptr) { return nullptr; }
            return merged_block_ptrs_[block_ptr->merged_block_id_];
        }

        bool lineCrossObstacleMergedBlock(const Pointi<N>& pt1, const Pointi<N>& pt2, const IS_OCCUPIED_FUNC<N>& is_occupied
                //,Pointis<N>& visited_pt
                , int& count_of_block
        ) {
            //if(isOutOfBoundary(pt1, dim_) || isOutOfBoundary(pt2, dim_)) { return true; }
            if(pt1 == pt2) return is_occupied(pt1);
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
                const auto& block_ptr = getInternalBlockPtr(pt);
                // if in block, jump over current block
                if(block_ptr != nullptr) {
                    assert(block_ptr->merged_block_id_ != -1);
#if 0
                    jump_step = findExitPointOfBlock(line, pt, i, static_cast<BlockPtr<N>>(block_ptr));
#else
                    const auto& merged_block_ptr = merged_block_ptrs_[block_ptr->merged_block_id_];
                    jump_step = findExitPointOfBlock(line, pt, i, merged_block_ptr->min_pt_, merged_block_ptr->max_pt_);
#endif
                    //std::cout << " jump step " << jump_step << std::endl;
//                    i = i + std::max(1, jump_step-1);
                    i = i + jump_step;
                    count_of_block ++;
                }
            }
            return false;
        }

        // given current pt's depth, get which child node contain pt
        // the top level's depth is 1, the deepest level's depth is max_depth
        size_t getIndex(const Pointi<N>& pt, int depth) const {
            assert(depth >= 0 && depth < max_depth_);
            size_t index = 0;
            int val1 = pow_2_[max_depth_-depth-1], val2 = pow_2_[max_depth_ - depth];
            //std::cout << "val1 = " << val1 << ", val2 = " << val2 << std::endl;
            for(int i=0; i<N; i++) {
                int indicator = (pt[i] % val2 / val1);
                //std::cout << "indicator = " << indicator << std::endl;
                assert(indicator == 1 || indicator == 0);
                index = index + indicator * pow_2_[i];
                //std::cout << "index = " << index << std::endl;
            }
            //std::cout << "pt = " << pt << ", depth = " << depth << ", index = " << index << std::endl;
            return index;
        }

        void printTree() const {
            std::cout << "-- " << __FUNCTION__ << std::endl;
            TreeNodePtrs<N> nodes = { root_ }, next_nodes;
            int dp = 0;
            while (!nodes.empty()) {
                std::cout << " depth = " << dp << ": " << std::endl;
                for(int i=0; i<nodes.size(); i++) {
                    assert(nodes[i]->depth_ == dp);
                    assert(nodes[i]->children_.size() == pow_2_[N]);
                    std::cout << nodes[i] << "(occ:" << nodes[i]->occ_
                              << ", mixed_state:" << nodes[i]->mixed_state_
                              << ", depth:" << nodes[i]->depth_
                              << ", base_pt:" << nodes[i]->base_pt_
                              << ")" << "->";
                    for(int j=0; j<pow_2_[N]; j++) {
                        if(nodes[i]->children_[j] != nullptr) {
                            std::cout << nodes[i]->children_[j]
                                      << "(occ:" << nodes[i]->children_[j]->occ_
                                      << ", mixed_state:" << nodes[i]->children_[j]->mixed_state_ << ") / ";
                            next_nodes.push_back(nodes[i]->children_[j]);
                        }
                    }
                    std::cout << std::endl;
                }
                nodes.clear();
                std::swap(nodes, next_nodes);
                dp ++;
            }
        }

        TreeNodePtrs<N> getAllPassableLeafNodes() const {
            //std::cout << "-- " << __FUNCTION__ << std::endl;
            TreeNodePtrs<N> nodes = { root_ }, next_nodes, retv;
            int dp = 0;
            while (!nodes.empty()) {
                //std::cout << " depth = " << dp << ": " << std::endl;
                for(int i=0; i<nodes.size(); i++) {
                    // assert(nodes[i]->depth_ == dp);
                    // assert(nodes[i]->children_.size() == pow_2_[N]);
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

        float getObstacleDensity() const {
            // count passable children grid
            float count = 0;
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            for(int i=0; i<total_index; i++) {
                Pointi<N> pt = IdToPointi<N>(i, dim_);
                if(!getInternalOccState(pt)) {
                    count ++;
                }
            }
            return count / getTotalIndexOfSpace<N>(dim_);
        }

        Pointi<N> all_1_pt_; // a point that all coordinate is 1
        Pointi<N> all_external_offset_;
        IS_OCCUPIED_FUNC<N> isoc_dynamic_; // notice, set state will change it


        IS_OCCUPIED_FUNC<N> isoc_; // notice, setOccupiedState will not change it,
        // so it is wrong after call setOccupiedState after initialize

        DimensionLength* dim_;

        TreeNodePtr<N> root_ = nullptr;

        int max_depth_ = 0;

        int min_block_depth_width_ = 1; // the minimum block width is pow(2, min_block_depth_width_)

        int external_min_block_depth_width_ = 0;

        std::vector<int> pow_2_; // precomputation of pow(2, x)

        Pointis<N> flag_pts_; // precomputation of all flag points

        bool initialized_ = false; // enable update block ptr only after initialized

        MergedBlockPtrs<N> merged_block_ptrs_;

    };


    template<Dimension N>
    using SpaceBinaryTreeRawPtr = std::shared_ptr<SpaceBinaryTreeRaw<N> >;



    // store all passable block
    // if a block (tree node)'s all children node is nullptr, it is passable
    // otherwise, some part of it are passable and others are occupied,
    // those passable are non-null, unpassable are null
    template<Dimension N>
    class SpaceBinaryTreeAnyDimensionRaw : public SpaceBinaryTreeRaw<N> {
    public:

        SpaceBinaryTreeAnyDimensionRaw(const IS_OCCUPIED_FUNC<N>& isoc, DimensionLength* dim,
                                       int min_block_depth_width = 1,
                                       int external_min_block_depth_width = 0)
                : SpaceBinaryTreeRaw<N>(isoc, dim, min_block_depth_width, external_min_block_depth_width) {
            Id total_index = getTotalIndexOfSpace<N>(this->dim_);
            occ_map_.resize(total_index, true);
            block_ptr_map_.resize(total_index, nullptr);
        }

        virtual void setInternalOccState(const Pointi<N>& pt, bool occ_state) override {
            if(isOutOfBoundary(pt, this->dim_)) { return; }
            occ_map_[PointiToId(pt, this->dim_)] = occ_state;
        }

        virtual bool getInternalOccState(const Pointi<N>& pt) const override {
            if(isOutOfBoundary(pt, this->dim_)) { return true; }
            return occ_map_[PointiToId(pt, this->dim_)];
        }

        virtual void setInternalBlockPtr(const Pointi<N>& pt, const BlockWithTreePtr<N>& block_ptr) override {
            if(isOutOfBoundary(pt, this->dim_)) { return; }
            block_ptr_map_[PointiToId(pt, this->dim_)] = block_ptr;
        }

        virtual const BlockWithTreePtr<N>& getInternalBlockPtr(const Pointi<N>& pt) const override {
            if(isOutOfBoundary(pt, this->dim_)) { return nullptr; }
            return block_ptr_map_[PointiToId(pt, this->dim_)];
        }

        virtual void clearInternalBlockPtr() {
            block_ptr_map_ = BlockWithTreePtrs<N>(getTotalIndexOfSpace<N>(this->dim_), nullptr);
        }

        BlockWithTreePtrs<N> block_ptr_map_; // save all grid's block ptr need lots space, but reduce time cost

        std::vector<bool> occ_map_; // save all grid's state need lots space, but reduce time cost


    };

    class SpaceBinaryTree2DRaw : public SpaceBinaryTreeRaw<2> {
    public:

        SpaceBinaryTree2DRaw(const IS_OCCUPIED_FUNC<2>& isoc, DimensionLength* dim,
                             int min_block_depth_width = 4,
                             int external_min_block_depth_width = 0)
                : SpaceBinaryTreeRaw<2>(isoc, dim, min_block_depth_width, external_min_block_depth_width) {
            //std::vector<bool> base_occ_map(dim[1], true);
            occ_map_.resize(dim[0]*dim[1], true);
            //std::vector<BlockPtrRaw<2> > base_block_map(dim[1], nullptr);
            block_ptr_map_.resize(dim[0]*dim[1], nullptr);
            std::cout << "start initialize of SBT2D" << std::endl;
        }

        virtual void setInternalOccState(const Pointi<2>& pt, bool occ_state) override {
            if(isOutOfBoundary(pt, this->dim_)) { return; }
            occ_map_[pt[0] + pt[1]*dim_[0]] = occ_state;
        }

        virtual bool getInternalOccState(const Pointi<2>& pt) const override {
//            if(isOutOfBoundary(pt, this->dim_)) { return true; }
            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1]) { return true; }
            auto index = pt[1]*dim_[0] + pt[0];
            return occ_map_[index];
        }

        virtual void setInternalBlockPtr(const Pointi<2>& pt, const BlockWithTreePtr<2>& block_ptr) override {
            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1]) { return; }
            block_ptr_map_[pt[0] + pt[1]*dim_[0]] = block_ptr;
        }

        virtual const BlockWithTreePtr<2>& getInternalBlockPtr(const Pointi<2>& pt) const override {
            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1]) { return nullptr; }
            return block_ptr_map_[pt[0] + pt[1]*dim_[0]];
        }

        virtual void clearInternalBlockPtr() {
            block_ptr_map_ = BlockWithTreePtrs<2>(getTotalIndexOfSpace<2>(dim_), nullptr);
        }

        BlockWithTreePtrs<2> block_ptr_map_; // save all grid's block ptr need lots space, but reduce time cost

        std::vector<bool> occ_map_; // save all grid's state need lots space, but reduce time cost


    };


    class SpaceBinaryTree3DRAW : public SpaceBinaryTreeRaw<3> {
    public:

        SpaceBinaryTree3DRAW(const IS_OCCUPIED_FUNC<3>& isoc, DimensionLength* dim,
                             int min_block_depth_width = 3,
                             int external_min_block_depth_width = 0)
                : SpaceBinaryTreeRaw<3>(isoc, dim, min_block_depth_width, external_min_block_depth_width) {
            //std::vector<bool> base_occ_map(dim[1], true);
            occ_map_.resize(dim[0]*dim[1]*dim[2], true);
            //std::vector<BlockPtrRaw<2> > base_block_map(dim[1], nullptr);
            block_ptr_map_.resize(dim[0]*dim[1]*dim[2], nullptr);
            std::cout << "start initialize of SBT3D" << std::endl;
        }

        virtual void setInternalOccState(const Pointi<3>& pt, bool occ_state) override {
            if(isOutOfBoundary(pt, this->dim_)) { return; }
//            occ_map_[PointiToId(pt, dim_)] = occ_state;
            occ_map_[pt[0] + pt[1]*dim_[0] + pt[2]*dim_[0]*dim_[1]] = occ_state;
        }

        virtual bool getInternalOccState(const Pointi<3>& pt) const override {
            if(isOutOfBoundary(pt, this->dim_)) { return true; }
//            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1] || pt[2] < 0 || pt[2] >= dim_[2]) { return true; }
            auto index = pt[1]*dim_[0] + pt[0] + pt[2]*dim_[0]*dim_[1];
            return occ_map_[index];
//            return occ_map_[PointiToId(pt, dim_)];
        }

        virtual void setInternalBlockPtr(const Pointi<3>& pt, const BlockWithTreePtr<3>& block_ptr) override {
//            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1] || pt[2] < 0 || pt[2] >= dim_[2]) { return ; }
            block_ptr_map_[pt[0] + pt[1]*dim_[0] + pt[2]*dim_[0]*dim_[1]] = block_ptr;
            //if(isOutOfBoundary(pt, this->dim_)) { return ; }
            //block_ptr_map_[PointiToId(pt, dim_)] = block_ptr;
        }

        virtual const BlockWithTreePtr<3>& getInternalBlockPtr(const Pointi<3>& pt) const override {
//            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1] || pt[2] < 0 || pt[2] >= dim_[2]) { return nullptr; }
            return block_ptr_map_[pt[0] + pt[1]*dim_[0] + pt[2]*dim_[0]*dim_[1]];
//            if(isOutOfBoundary(pt, this->dim_)) { return nullptr; }
//            return block_ptr_map_[PointiToId(pt, dim_)];
        }

        virtual void clearInternalBlockPtr() {
            block_ptr_map_ = BlockWithTreePtrs<3>(getTotalIndexOfSpace<3>(dim_), nullptr);
        }

        BlockWithTreePtrs<3> block_ptr_map_; // save all grid's block ptr need lots space, but reduce time cost

        std::vector<bool> occ_map_; // save all grid's state need lots space, but reduce time cost


    };

}

#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_RAW_H
