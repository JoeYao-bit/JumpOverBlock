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

    template<Dimension N>
    using TreeNodePtr = std::shared_ptr<TreeNode<N> >;

    template<Dimension N>
    using TreeNodePtrs = std::vector<TreeNodePtr<N> >;

//    template<Dimension N>
//    using BlockPtrRaw = Block<N>*;

    template<Dimension N>
    struct TreeNodeNew;

    template<Dimension N>
    using TreeNodeNewPtr = std::shared_ptr<TreeNodeNew<N> >;

    template<Dimension N>
    using TreeNodeNewPtrs = std::vector<TreeNodeNewPtr<N> >;

    template<Dimension N>
    struct BlockRaw : public Block<N> {
        TreeNodeNewPtr<N> tree_node_ = nullptr;
    };

    template<Dimension N>
    using BlockPtrRaw = std::shared_ptr<BlockRaw < N> >;

    template<Dimension N>
    using BlockPtrsRaw = std::vector<BlockPtrRaw<N> >;



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
        // the value has meaning only when mixed_state_ = false
        bool occ_ = true;

        int depth_ = 0; // for debug, can be removed when everything is okay

        Pointi<N> base_pt_; // the minimum point of the cube of current node

//        BlockPtrRaw<N> block_ptr_ = nullptr;

    };





    // all leaf node's children is all nullptr
    // if a node have non-nullptr children, it has a mixed state, part of it is passable and other part is unpassable
    template<Dimension N>
    struct TreeNodeNew {

        explicit TreeNodeNew(TreeNodeNewPtr<N> parent = nullptr) : parent_(parent) {
            if(parent_ != nullptr) {
                depth_ = parent_->depth_ + 1;
            }
        }

        // all leaf node's are occupied or unpassable, mixed_state = false
        // otherwise, mixed_state = true
        // when a node is mixed state, it's occ_ = true or false is meaningless
        bool mixed_state_ = false;

        TreeNodeNewPtr<N> parent_ = nullptr;

        TreeNodeNewPtrs<N> children_; // have 2^N child node, fixed size, if all null, they all are the same state as this block

        // location of pose and and depth can be get from accumulation of index of children and depth

        // if is not a mixed_state, but a leaf node, whether it is occupied
        // the value has meaning only when mixed_state_ = false
        bool occ_ = true;

        int depth_ = 0; // for debug, can be removed when everything is okay

        Pointi<N> base_pt_; // the minimum point of the cube of current node

//        BlockPtrRaw<N> block_ptr_ = nullptr;

    };

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


}

#endif //JUMPOVERBLOCK_COMMON_H
