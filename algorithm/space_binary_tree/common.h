//
// Created by yaozhuo on 7/22/25.
//

#ifndef JUMPOVERBLOCK_COMMON_H
#define JUMPOVERBLOCK_COMMON_H

#include <auto_ptr.h>
#include <vector>
#include "../../freeNav-base/basic_elements/point.h"

namespace freeNav::JOB {

    template<Dimension N>
    struct TreeNode;

//    template<Dimension N>
//    using TreeNodePtr = std::shared_ptr<TreeNode<N> >;

    template<Dimension N>
    using TreeNodePtr = TreeNode<N>*;

    template<Dimension N>
    using TreeNodePtrs = std::vector<TreeNodePtr<N> >;

//    template<Dimension N>
//    using BlockPtrRaw = Block<N>*;

//    template<Dimension N>
//    struct TreeNodeNew;
//
//    template<Dimension N>
//    using TreeNodeNewPtr = std::shared_ptr<TreeNodeNew<N> >;
//
//    template<Dimension N>
//    using TreeNodeNewPtrs = std::vector<TreeNodeNewPtr<N> >;

    template<Dimension N>
    struct BlockWithTree : public Block<N> {

        TreeNodePtr<N> tree_node_ = nullptr;
        int merged_block_id_  = -1;

    };

    template<Dimension N>
    struct BlockWithTreeRaw : public Block<N> {

        TreeNodePtr<N> tree_node_ = nullptr;
        int merged_block_id_  = -1;

    };

    template<Dimension N>
    using BlockWithTreePtr = std::shared_ptr<BlockWithTree < N> >;

    template<Dimension N>
    using BlockWithTreePtrs = std::vector<BlockWithTreePtr<N> >;



    // all leaf node's children is all nullptr
    // if a node have non-nullptr children, it has a mixed state, part of it is passable and other part is unpassable
    template<Dimension N>
    struct TreeNode {

        explicit TreeNode(TreeNodePtr<N> parent) : parent_(parent) {
            children_.resize(pow(2, N), nullptr);
            for(int i=0; i<pow(2, N); i++) {
                children_[i] = nullptr;
            }
            if(parent_ != nullptr) {
                depth_ = parent_->depth_ + 1;
            }
        }

        explicit TreeNode() : parent_(nullptr) {
            children_.resize(pow(2, N), nullptr);
            for(int i=0; i<pow(2, N); i++) {
                children_[i] = nullptr;
            }
        }

        ~ TreeNode() {
//            std::cout << "N = " << N << std::endl;
//            std::cout << "children_.size() = " << children_.size() << std::endl;
            if(children_.size() == pow(2, N)) {
                for(int i=0; i<pow(2, N); i++) {
                    if(children_[i] != nullptr) {
                        delete children_[i];
                        children_[i] = nullptr;
                    }
                }
            }
        }

        // all leaf node's are occupied or unpassable, mixed_state = false
        // otherwise, mixed_state = true
        // when a node is mixed state, it's occ_ = true or false is meaningless
        bool mixed_state_ = false;

        int id_; // id in tree node vec

        TreeNodePtr<N> parent_ = nullptr;

        TreeNodePtrs<N> children_; // have 2^N child node, fixed size, if all null, they all are the same state as this block

        // location of pose and and depth can be get from accumulation of index of children and depth

        // if is not a mixed_state, but a leaf node, whether it is occupied
        // the value has meaning only when mixed_state_ = false
        bool occ_ = true;

        int depth_ = 0; // for debug, can be removed when everything is okay

        Pointi<N> base_pt_; // the minimum point of the cube of current node

//        BlockPtrRaw<N> block_ptr_ = nullptr;

    };





//    // all leaf node's children is all nullptr
//    // if a node have non-nullptr children, it has a mixed state, part of it is passable and other part is unpassable
//    template<Dimension N>
//    struct TreeNodeNew {
//
//        explicit TreeNodeNew(TreeNodeNewPtr<N> parent = nullptr) : parent_(parent) {
//            if(parent_ != nullptr) {
//                depth_ = parent_->depth_ + 1;
//            }
//        }
//
//        // all leaf node's are occupied or unpassable, mixed_state = false
//        // otherwise, mixed_state = true
//        // when a node is mixed state, it's occ_ = true or false is meaningless
//        bool mixed_state_ = false;
//
//        int id_; // id in tree node vec
//
//        TreeNodeNewPtr<N> parent_ = nullptr;
//
//        TreeNodeNewPtrs<N> children_; // have 2^N child node, fixed size, if all null, they all are the same state as this block
//
//        // location of pose and and depth can be get from accumulation of index of children and depth
//
//        // if is not a mixed_state, but a leaf node, whether it is occupied
//        // the value has meaning only when mixed_state_ = false
//        bool occ_ = true;
//
//        int depth_ = 0; // for debug, can be removed when everything is okay
//
//        Pointi<N> base_pt_; // the minimum point of the cube of current node
//
////        BlockPtrRaw<N> block_ptr_ = nullptr;
//
//    };

    //    // return: current in the block or not
//    // for a line that cross a block, find the point on it and leave obstacle
//    // update inner index of line
//    template<Dimension N>
//    int findExitPointOfBlock(Line<N>& line, const Pointi<N>& current_pt, const int& index, const BlockPtrRaw<N>& block_ptr) {
//        // check whether the line reach end of line
//        if(index >= line.step - 1) {return 0; }
//        //Pointi<N> current_pt = line.GetPoint(index);
//        // check whether current line's last traveled point in the block
//        //if(!block_ptr->PointiInBlock(current_pt)) {
//        //std::cout << " not in block" << std::endl;
//        //    return 0;
//        //}
//        //bool line_increase = (line.step_length > 0);
//        //Dimension minimum_step_exit_dim = 0;
//        Fraction minimum_step_to_exit = Fraction(line.step), future_step;
//        // determine the fast dim to leave current block
//        for(Dimension dim=0; dim<N; dim++) {
//            if(line.parameter[dim].second == 0) {
//                continue;
//            } else {
//                if(line.parameter[dim].second > 0) {
//                    future_step = (Fraction(block_ptr->max_[dim] - current_pt[dim]) /
//                                   line.parameter[dim].second).toAbs();
//                } else {
//                    future_step = (Fraction(block_ptr->min_[dim] - current_pt[dim]) /
//                                   line.parameter[dim].second).toAbs();
//                }
//                //std::cout << " dim " << dim << " / future_step " << future_step.toFloat() << std::endl;
//                if (future_step < minimum_step_to_exit) {
//                    //minimum_step_exit_dim = dim;
//                    minimum_step_to_exit = future_step;
//                }
//            }
//        }
//        //std::cout << " line.step " << line.step << " - index " << index << std::endl;
//        //std::cout << " minimum_step_to_exit " << minimum_step_to_exit << std::endl;
//        if(minimum_step_to_exit > line.step - index) {
//            //std::cout << " reach end of line" << std::endl;
//            return line.step - index - 1;
//        }
//        return std::max((line.step*minimum_step_to_exit - 1).floor(), 0);
//    }


    template<Dimension N>
    struct MergedBlock;

    template<Dimension N>
    using MergedBlockPtr = std::shared_ptr<MergedBlock<N> >;

    template<Dimension N>
    using MergedBlockPtrs = std::vector<MergedBlockPtr<N> >;

    template<Dimension N>
    struct MergedBlock {
        Pointi<N> min_pt_;
        Pointi<N> max_pt_;

        Pointi<N> min_pt_ex_;
        Pointi<N> max_pt_ex_;

        BlockWithTreePtrs<N> block_ptrs_; // leaf node of SBT in current block

//        MergedBlockPtr<N> parent_;
//        MergedBlockPtrs<N> children_;
    };

    template<Dimension N>
    struct BlockCompare {
        bool operator()(const BlockWithTreePtr<N>& a, const BlockWithTreePtr<N>& b) const {
            assert(a->tree_node_ != nullptr && b->tree_node_ != nullptr);
            if(a->tree_node_->depth_ != b->tree_node_->depth_) {
                return a->tree_node_->depth_ < b->tree_node_->depth_;
            } else {
                for(int d=0; d<N; d++) {
                    if(a->tree_node_->base_pt_[d] != b->tree_node_->base_pt_[d]) {
                        return a->tree_node_->base_pt_[d] < b->tree_node_->base_pt_[d];
                    }
                }
                //std::cout << " there should not have the same leaf node" << std::endl;
                //assert(0);
                // 当 comp(a, b) == false 且 comp(b, a) == false 时
                //std::map 会认为 a 和 b 等价（即“相同”的 key），此时插入操作会覆盖已有值或失败（取决于插入方式）。
                return a->tree_node_->base_pt_[N-1] < b->tree_node_->base_pt_[N-1];
            }
        }
    };

}

#endif //JUMPOVERBLOCK_COMMON_H
