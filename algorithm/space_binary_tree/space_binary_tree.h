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
            for(int i=0; i<pow(2, N); i++) {
                children_[i] = nullptr;
            }
        }

        bool all_child_null_ = true;

        TreeNodePtr<N> parent_ = nullptr;

        TreeNodePtr<N> children_[2^N]; // have 2^N child node, fixed size

        // location of pose and and depth can be get from accumulation of index of children and depth

    };

    // store all passable block
    // if a block (tree node)'s all children node is nullptr, it is passable
    // otherwise, some part of it are passable and others are occupied
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
                if(pow(2, max_depth_) < max_depth_) {
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

        }

        // update state of node, do not update isoc
        void setPassable(const Pointi<N>& pt) {
            // if already passable, do nothing

        }

        // a point's index at last level, if want get index at depth, replace pt with pt/pow(2, max_depth - depth)
        // the top level's depth is 1, the deepest level's depth is max_depth
        size_t getIndex(const Pointi<N>& pt, int depth) const {
            size_t index = 0;
            for(int i=0; i<N; i++) {
                //id[i] = pt[i] / 2;
                index = index + (pt[i] / pow(2, max_depth_ - depth + 1)) * pow(2, i);
            }
            return index;
        }

        IS_OCCUPIED_FUNC<N> isoc_;

        DimensionLength* dim_;

        TreeNodePtr<N> root_ = nullptr;

        int max_depth_ = 0;

        //TreeNodePtrs<N> all_nodes_; // need lots space

    };

}

#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_H
