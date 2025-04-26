//
// Created by yaozhuo on 2025/4/24.
//

#ifndef JUMPOVERBLOCK_SPACE_BINARY_TREE_H
#define JUMPOVERBLOCK_SPACE_BINARY_TREE_H

#include <auto_ptr.h>
#include <vector>
#include "freeNav-base/basic_elements/point.h"

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
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            for(Id id=0; id<total_index; id++) {
                Pointi<N> pt = IdToPointi<N>(id, dim_);
                setOccupiedState(pt, isoc_(pt));
            }
        }

        // update state of node, do not update isoc
        // set passable to unpassable may result new tree node and erase existing node
        void setOccupiedState(const Pointi<N>& pt, bool is_occupied) {
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
                    for(; dp<max_depth_; dp++) {
                        for(int i=0; i<pow(2, N); i++) {
                            buffer->children_[i] = std::make_shared<TreeNode<N> >(buffer);
                            buffer->children_[i]->occ_ = !is_occupied;
                            buffer->children_[i]->mixed_state_ = false;
                        }
                        size_t index = getIndex(pt, dp);
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
        void recurAndUpdate(TreeNodePtr<N> parent) {
            //std::cout << "recurAndUpdate" << std::endl;
            assert(parent->depth_ == max_depth_ - 1);
            // remove child node if all child are passable
            for(int dp=max_depth_-1; dp>=0; dp--) {
                assert(parent != nullptr);
                bool all_same_state = true;
                // check whether all child is the same state,
                // if is the same state, set all child to nullptr and mixed_state to false
                for(int i=0; i<pow(2, N); i++) {
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
                    for(int i=0; i<pow(2, N); i++) {
                        parent->children_[i] = nullptr;
                    }
                } else {
                    parent->mixed_state_ = true;
                }
                parent = parent->parent_;
            }
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
            exit(0);
            return true;
        }

        // given current pt's depth, get which child node contain pt
        // the top level's depth is 1, the deepest level's depth is max_depth
        size_t getIndex(const Pointi<N>& pt, int depth) const {
            assert(depth >= 0 && depth < max_depth_);
            size_t index = 0;
            int val1 = pow(2, max_depth_-depth-1), val2 = pow(2, max_depth_ - depth);
            //std::cout << "val1 = " << val1 << ", val2 = " << val2 << std::endl;
            for(int i=0; i<N; i++) {
                int indicator = (pt[i] % val2 / val1);
                //std::cout << "indicator = " << indicator << std::endl;
                assert(indicator == 1 || indicator == 0);
                index = index + indicator * pow(2, i);
                //std::cout << "index = " << index << std::endl;
            }
            //std::cout << "pt = " << pt << ", depth = " << depth << ", index = " << index << std::endl;
            return index;
        }

        void printTree() const {
            TreeNodePtrs<N> nodes = { root_ }, next_nodes;
            int dp = 0;
            while (!nodes.empty()) {
                std::cout << " depth = " << dp << ": " << std::endl;
                for(int i=0; i<nodes.size(); i++) {
                    assert(nodes[i]->depth_ == dp);
                    assert(nodes[i]->children_.size() == pow(2, N));
                    std::cout << nodes[i] << "(occ:" << nodes[i]->occ_
                              << ", mixed_state:" << nodes[i]->mixed_state_ << ")" << "->";
                    for(int j=0; j<pow(2, N); j++) {
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

        IS_OCCUPIED_FUNC<N> isoc_;

        DimensionLength* dim_;

        TreeNodePtr<N> root_ = nullptr;

        int max_depth_ = 0;

        //TreeNodePtrs<N> all_nodes_; // need lots space

    };

}

#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_H
