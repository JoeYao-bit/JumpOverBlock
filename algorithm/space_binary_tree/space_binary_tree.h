//
// Created by yaozhuo on 2025/4/24.
//

#ifndef JUMPOVERBLOCK_SPACE_BINARY_TREE_H
#define JUMPOVERBLOCK_SPACE_BINARY_TREE_H

#include <auto_ptr.h>
#include <vector>
#include "freeNav-base/basic_elements/point.h"
#include "../line_of_sight_jump_between_block.h"

namespace freeNav::JOB {

    template<Dimension N>
    struct TreeNode;

    template<Dimension N>
    using TreeNodePtr = std::shared_ptr<TreeNode<N> >;

    template<Dimension N>
    using TreeNodePtrs = std::vector<TreeNodePtr<N> >;

    // all leaf node's children is all nullptr
    // if a node have non-nullptr children, it has a mixed state, part of it is passable and other part is unpassable
    template<Dimension N>
    struct TreeNode {

        explicit TreeNode(TreeNodePtr<N> parent = nullptr) : parent_(parent) {
            children_.resize(pow(2, N), nullptr);
            for(int i=0; i<pow(2, N); i++) {
                children_[i] = nullptr;
            }
            if(parent_ != nullptr) {
                depth_ = parent_->depth_ + 1;
            }
        }

        // all leaf node's are occupied or unpassable, mixed_state = false
        // otherwise, mixed_state = true
        // when a node is mixed state, it's occ_ = true or false is meaningless
        bool mixed_state_ = false;

        TreeNodePtr<N> parent_ = nullptr;

        TreeNodePtrs<N> children_; // have 2^N child node, fixed size, if all null, they all are the same state as this block

        // location of pose and and depth can be get from accumulation of index of children and depth

        // if is not a mixed_state, but a leaf node, whether it is occupied
        bool occ_ = true;

        int depth_ = 0; // for debug, can be removed when everything is okay

        Pointi<N> base_pt_; // the minimum point of the cube of current node

    };

    // store all passable block
    // if a block (tree node)'s all children node is nullptr, it is passable
    // otherwise, some part of it are passable and others are occupied,
    // those passable are non-null, unpassable are null
    template<Dimension N>
    class SpaceBinaryTree {
    public:

        SpaceBinaryTree(const IS_OCCUPIED_FUNC<N>& isoc, DimensionLength* dim)
        : isoc_(isoc), dim_(dim) {
            // initialize
            root_ = std::make_shared<TreeNode<N> >();
            root_->base_pt_ = Pointi<N>();
            // get max dimension length
            DimensionLength max_dim = 0;
            for(int i=0; i<N; i++) {
                max_dim = std::max(max_dim, dim_[i]);
            }
            std::cout << "max_dim_length = " << max_dim << std::endl;
            max_depth_ = 1;
            while(true) {
                if(pow(2, max_depth_) < max_dim) {
                    max_depth_ ++;
                } else {
                    break;
                }
            }
            std::cout << "max_depth = " << max_depth_ << std::endl;
            // precomputation of pow(2, x)
            for(int dp=0; dp<=std::max(max_depth_, (int)N); dp++) {
                pow_2_.push_back(pow(2, dp));
            }
            // precomputation of flag points
            flag_pts_ = GetFloorOrCeilFlag<N>();
            assert(flag_pts_.size() == pow_2_[N]);
            // initialize of space
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            block_ptr_map_.resize(total_index, nullptr);
            occ_map_.resize(total_index, true);

            initialized_ = false;
            root_->occ_ = true;
            for(Id id=0; id<total_index; id++) {
                Pointi<N> pt = IdToPointi<N>(id, dim_);
                if(!isoc_(pt)) {
                    setOccupiedState(pt, false);
                    occ_map_[id] = false;
                }
            }
            // initialize of block_ptr_map_
            std::vector<TreeNodePtr<N> > free_leaf_nodes = getAllPassableLeafNodes();
            for(const auto& leaf_node : free_leaf_nodes) {
                if(leaf_node->depth_ >= max_depth_ - min_block_depth_width_) { continue; } // limit minimum size of blocks
                BlockPtr<N> block_ptr = std::make_shared<Block<N> >();
                block_ptr->min_ = leaf_node->base_pt_;
                Pointi<N> offset; offset.setAll(pow_2_[max_depth_-leaf_node->depth_]-1);
                block_ptr->max_ = leaf_node->base_pt_ + offset;
                setBlockPtrForNode(leaf_node, block_ptr);
            }
            initialized_ = true;
        }

        // set all grid in current node range to the same block_ptr
        void setBlockPtrForNode(const TreeNodePtr<N>& node, const BlockPtr<N>& block_ptr) {
            assert(!block_ptr_map_.empty());
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
                global_id = PointiToId(global_pt, dim_);
                block_ptr_map_[global_id] = block_ptr;
            }
        }

        // update state of node, do not update isoc
        // set passable to unpassable may result new tree node and erase existing node
        // and update block_ptr_map_
        void setOccupiedState(const Pointi<N>& pt, bool is_occupied) {
            if(isOutOfBoundary(pt, dim_)) { return ; }
            Id id = PointiToId(pt, dim_);
            occ_map_[id] = is_occupied;

            TreeNodePtr<N> buffer = root_;
            for(int dp=0; dp <= max_depth_; dp++) {
                //std::cout << "buffer->depth_ = " << buffer->depth_ << std::endl;
                if(!buffer->mixed_state_) {
                    // if reach a leaf node
                    if(buffer->occ_ == is_occupied) {
                        // if current block is already the same state, do nothing
//                        std::cout << "leaf node has the same state, do nothing" << std::endl;
                        return;
                    }
                    // reach a leaf node, and its not the same state
                    //std::cout << "reach a leaf node, and its not the same state" << std::endl;
                    buffer->mixed_state_ = true;
                    // update block_ptr_map_
                    if(is_occupied && initialized_) {
                        // set all block ptr in current leaf node as nullptr
                        setBlockPtrForNode(buffer, nullptr);
                    }
                    for(; dp<max_depth_; dp++) {
                        size_t index = getIndex(pt, dp);
                        for(int i=0; i<pow_2_[N]; i++) {
                            buffer->children_[i] = std::make_shared<TreeNode<N> >(buffer);
                            int zoom_ratio = pow_2_[max_depth_-dp-1];
                            buffer->children_[i]->base_pt_ = buffer->base_pt_ + flag_pts_[i].multi(zoom_ratio);
                            buffer->children_[i]->occ_ = !is_occupied;
                            buffer->children_[i]->mixed_state_ = false;
                            // set block ptr when is_occupied = true, as this action may create multiple small blocks
                            // as when is_occupied = true, no block ptr will be set in recurAndUpdate
                            if(is_occupied && i != index &&
                                    (buffer->children_[i]->depth_ < max_depth_ - min_block_depth_width_)) {
                                BlockPtr<N> block_ptr = std::make_shared<Block<N> >();
                                block_ptr->min_ = buffer->children_[i]->base_pt_;
                                Pointi<N> offset; offset.setAll(pow_2_[max_depth_-buffer->children_[i]->depth_]-1);
                                block_ptr->max_ = buffer->children_[i]->base_pt_ + offset;
                                //std::cout << "create block min/max " << block_ptr->min_ << ", " << block_ptr->max_ << std::endl;
                                setBlockPtrForNode(buffer->children_[i], block_ptr);
                            }
                        }
                        buffer = buffer->children_[index];
                        buffer->mixed_state_ = true;
                    }
                    buffer->occ_ = is_occupied;
                    buffer->mixed_state_ = false; // last node is leaf node, it is not mixed state
                    break;
                } else {
                    size_t index = getIndex(pt, dp);
                    buffer = buffer->children_[index];
                }
            }
            //printTree();
            // if a node's all children node is passable, set all it's children node to nullptr
            TreeNodePtr<N> parent = buffer->parent_;
            recurAndUpdate(parent);
        }

        // set mixed_state to false and all children to nullptr if all child are occupied or unpassable
        // and update block_ptr_map_
        void recurAndUpdate(TreeNodePtr<N> parent) {
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
                        parent->children_[i] = nullptr;
                    }
                    if(initialized_) {
                        // if set to passable, check whether create big block
                        // limit minimum size of blocks
                        //std::cout << "create block, parent->occ_ = " << parent->occ_ << std::endl;
                        if(!parent->occ_ && (parent->depth_ < max_depth_ - min_block_depth_width_)) {
                            BlockPtr<N> block_ptr = std::make_shared<Block<N> >();
                            block_ptr->min_ = parent->base_pt_;
                            Pointi<N> offset; offset.setAll(pow_2_[max_depth_-parent->depth_]-1);
                            block_ptr->max_ = parent->base_pt_ + offset;
                            //std::cout << "create block min/max " << block_ptr->min_ << ", " << block_ptr->max_ << std::endl;
                            setBlockPtrForNode(parent, block_ptr);
                        }
                    }
                } else {
                    parent->mixed_state_ = true;
                }
                parent = parent->parent_;
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
//                    // construct a local BlockPtr and jump
//                    BlockPtr<N> block_ptr = std::make_shared<Block<N> >();
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

        bool lineCrossObstacle(const Pointi<N>& pt1, const Pointi<N>& pt2,
                               Pointis<N>& visited_pt,
                               int& count_of_block) const {
            if(isOutOfBoundary(pt1, dim_) || isOutOfBoundary(pt2, dim_)) { return true; }
            if(pt1 == pt2) return true;
            visited_pt.clear();
            count_of_block = 0;
            Line<N> line(pt1, pt2);
            int check_step = line.step;
            Pointi<N> pt;
            Id current_id, id;
            int jump_step = 0;
            for(int i=1; i<check_step; i++) {
                pt = line.GetPoint(i);
                visited_pt.push_back(pt);
                if(isoc_(pt)) {
                    return true;
                }
                //if(is_occupied(pt)) { return true; } // ignore dynamic update of map
                id = PointiToId(pt, dim_);
                if(occ_map_[id]) { return true; }
                current_id = PointiToId(pt, dim_);
                const auto& block_ptr = block_ptr_map_[current_id];
                // if in block, jump over current block
                if(block_ptr != nullptr) {
                    jump_step = findExitPointOfBlock(line, pt, i, block_ptr);
                    //std::cout << " jump step " << jump_step << std::endl;
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

        IS_OCCUPIED_FUNC<N> isoc_; // notice, setOccupiedState will not change it,
        // so it is wrong after call setOccupiedState after initialize

        DimensionLength* dim_;

        TreeNodePtr<N> root_ = nullptr;

        int max_depth_ = 0;

        int min_block_depth_width_ = 1; // the minimum block width is pow(2, min_block_depth_width_)

        std::vector<int> pow_2_; // precomputation of pow(2, x)

        Pointis<N> flag_pts_; // precomputation of all flag points

        BlockPtrs<N> block_ptr_map_; // save all grid's block ptr need lots space, but reduce time cost

        std::vector<bool> occ_map_; // save all grid's state need lots space, but reduce time cost

        bool initialized_ = false; // enable update block ptr only after initialized

    };

}

#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_H
