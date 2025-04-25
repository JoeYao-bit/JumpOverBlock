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

        // all leaf node's all_child_null_ = true
        bool all_child_null_ = true;

        TreeNodePtr<N> parent_ = nullptr;

        TreeNodePtrs<N> children_; // have 2^N child node, fixed size, if all null, they all are the same state as this block

        // location of pose and and depth can be get from accumulation of index of children and depth

        // occ_ = true if any of its child is occupied, occ = free means all_child_null_ = true;
        // but
        bool occ_ = true;

        int depth_ = 1; // for debug, should be removed when everything is okay

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
        }

        // update state of node, do not update isoc
        // set passable to unpassable may result new tree node and erase existing node
        void setOccupied(const Pointi<N>& pt) {
            // if already occupied, do nothing
            if(isOccupied(pt)) { return; }

            TreeNodePtr<N> buffer = root_;
            for(int dp=1; dp<=max_depth_; dp++) {
                if(buffer->all_child_null_) {
                    // if current block is all passable, create 2^N - 1 passable node and a single unpassable node
                    // till max depth

                    return;
                }
                // if current block is partly passable and partly unpassable, no need to change
                size_t index = getIndex(pt, dp);

            }
        }

        // update state of node, do not update isoc
        // d the leaf node that contain pt and set it till max depth
        void setPassable(const Pointi<N>& pt) {
            TreeNodePtr<N> buffer = root_;
            int key[max_depth_]; // store block index of current pt at all level
            for(int dp=1; dp < max_depth_; dp++) {
                // if current block is partly passable and partly unpassable, no need to change
                if(!buffer->occ) {
                    // if current block is already passable, do nothing
                    return;
                } else {
                    if(buffer->all_child_null_) {
                        // reach a leaf node, and its unpassable
                        for(; dp<max_depth_; dp++) {
                            for(int i=0; i<pow(2, N); i++) {
                                buffer->children_[i] = std::make_shared<TreeNode<N> >(buffer);
                                buffer->children_[i]->occ_ = buffer->occ_;
                            }
                            size_t index = getIndex(pt, dp);
                            buffer->children_[index]->occ_ = false;
                        }
                        buffer = buffer->children_[index];
                        break;
                    }
                }
                size_t index = getIndex(pt, dp);
                buffer = buffer->children_[index];
                if(dp == max_depth_ - 1) {
                    std::cout << "reach deepest level" << std::endl;
                    buffer->occ_ = false;
                    break;
                }
            }
            // if a node's all children node is passable, set all it's children node to nullptr
            TreeNodePtr<N> parent = buffer->parent_;
            assert(parent->depth_ == max_depth_ - 1);
            // remove child node if all child are passable
            for(int dp=max_depth_-1; dp>=1; dp--) {
                assert(parent != nullptr);
                bool all_passable = true;
                // check whether all child is passable
                for(int i=0; i<pow(2, N); i++) {
                    if(parent->children_[i]->occ_) {
                        all_passable = false;
                        break;
                    }
                }
                if(!all_passable) {
                    // if not all passable, continue
                    break;
                } else {
                    // if all is passable, remove all child node
                    for(int i=0; i<pow(2, N); i++) {
                        parent->children_[i] = nullptr;
                    }
                    parent->all_child_null_ = true;
                }
                parent = parent->parent_;
            }
        }

        // find the leaf node that contain current pt, and return its occ
        bool isOccupied(const Pointi<N>& pt) const {
            TreeNodePtr<N> buffer = root_;
            for(int dp=1; dp<max_depth_; dp++) {
                if(buffer->all_child_null_) {
                    return buffer->occ_;
                }
                size_t index = getIndex(pt, dp);
                buffer = buffer->children_[index];
            }
            return true;
        }

        // given current pt's depth, get which child node contain pt
        // the top level's depth is 1, the deepest level's depth is max_depth
        size_t getIndex(const Pointi<N>& pt, int depth) const {
            assert(depth >= 1 && depth < max_depth_);
            size_t index = 0;
            for(int i=0; i<N; i++) {
                index = index + (pt[i] / pow(2, max_depth_ - depth)) * pow(2, i);
            }
            return index;
        }

        void printTree() const {
            TreeNodePtrs<N> nodes = { root_ }, next_nodes;
            int dp = 1;
            while (!nodes.empty()) {
                std::cout << " depth = " << dp << ": " << std::endl;
                for(int i=0; i<nodes.size(); i++) {
                    assert(nodes[i]->depth_ == dp);
                    assert(nodes[i]->children_.size() == pow(2, N));
                    std::cout << nodes[i] << "(occ:" << nodes[i]->occ_
                                    << ", all_null:" << nodes[i]->all_child_null_ << ")" << "->";
                    for(int j=0; j<pow(2, N); j++) {
                        if(nodes[i]->children_[j] != nullptr) {
                            std::cout << nodes[i]->children_[j]
                                << "(occ:" << nodes[i]->children_[j]->occ_
                          << ", all_null:" << nodes[i]->children_[j]->all_child_null_ << ") / ";
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
