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

        SpaceBinaryTree(const IS_OCCUPIED_FUNC <N> &isoc, DimensionLength *dim, int min_block_depth_width = 0)
                : isoc_(isoc), dim_(dim), min_block_depth_width_(min_block_depth_width) {

            std::cout << "min_block_depth_width = " << min_block_depth_width_ << std::endl;

            // initialize
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
            for (int dp = 0; dp <= max_depth_*(int)N; dp++) {
                pow_2_.push_back(pow(2, dp));
            }
            // precomputation of flag points
            flag_pts_ = GetFloorOrCeilFlag<N>();
            assert(flag_pts_.size() == pow_2_[N]);

            int size_of_vec = 1;
            level_offset_ = {0};
            for(int d=1; d<=max_depth_; d++) {
                size_of_vec = size_of_vec + pow_2_[d*N];
                level_offset_.push_back(level_offset_.back() + pow(2, (d-1)*N));
            }
            std::cout << "size_of_vec = " << size_of_vec << std::endl;
            std::cout << " level_offset_ = ";
            for(int d=0; d<=max_depth_; d++) {
                std::cout << level_offset_[d] << " ";
            }
            std::cout << std::endl;

            tree_ptr_vec_.resize(size_of_vec, nullptr);
            std::cout << "flag 0" << std::endl;

            max_dims_.resize(max_depth_+1, nullptr);

            for(int dp=0; dp<=max_depth_; dp++) {
                int max_width = pow_2_[dp];
                max_dims_[dp] = new DimensionLength[N];
                for (int i = 0; i < N; i++) {
                    max_dims_[dp][i] = max_width;
                }
                //std::cout << "max_width at level " << dp << " = " << max_width << std::endl;
            }
            // NOTICE: in override class's constructor, set all internal occ map state to occupied
        }

        ~SpaceBinaryTree() {
            for(int dp=0; dp<=max_depth_; dp++) {
                delete max_dims_[dp];
                max_dims_[dp] = nullptr;
            }
        }

        int PtToTreeVecId(const Pointi<N>& pt, int level) const {
            Pointi<N> local_pt = pt / pow_2_[(max_depth_-level)];
            return level_offset_[level] + PointiToId(local_pt, max_dims_[level]);
        }

        void initialize() {
            // initialize max depth level tree node
            Pointi<N> pt;
            int max_id = getTotalIndexOfSpace<N>(max_dims_[max_depth_]);
            for(int id=0; id<max_id; id++) {
                pt = IdToPointi<N>(id, max_dims_[max_depth_]);
                auto new_tree_node = std::make_shared<TreeNodeNew<N> >(nullptr);
                new_tree_node->base_pt_ = pt;
                new_tree_node->occ_ = isoc_(pt);
                new_tree_node->mixed_state_ = false;
                new_tree_node->depth_ = max_depth_;
                tree_ptr_vec_[level_offset_[max_depth_] + id] = new_tree_node;

                setInternalOccState(pt, new_tree_node->occ_);
            }
            // from deep to top, construct parent node
            int child_id_global;
            for(int cur_level = max_depth_-1; cur_level>=0; cur_level--) {
                int cur_level_vec_offset = level_offset_[cur_level];
                int next_level_vec_offset = level_offset_[cur_level+1];
//                std::cout << "cur_level = " << cur_level <<  ", cur / next level_vec_offset = "
//                          << cur_level_vec_offset << " / " << next_level_vec_offset << std::endl;
                //std::cout << "pow_2_[cur_level] = " << pow_2_[cur_level] << std::endl;
                for(int id=0; id<pow_2_[cur_level*N]; id++) {
                    bool all_the_same = true;
                    int is_occ = -1;
                    Pointi<N> cur_pt = IdToPointi<N>(id, max_dims_[cur_level]);
//                    std::cout << "cur_pt = " << cur_pt << std::endl;
                    // number of child node os pow(2, N)
                    //std::cout << "next offset = " << next_level_vec_offset + id*pow_2_[N] << std::endl;
                    std::vector<Id> child_global_ids;
                    for(const auto& child_offset : flag_pts_) {
                        //std::cout << "child_id = " << child_id << std::endl;
                        Pointi<N> child_pt = cur_pt*2 + child_offset;
//                        std::cout << "child_pt = " << child_pt << std::endl;
//                        std::cout << "child pt local id = " << PointiToId<N>(child_pt, max_dims_[cur_level+1])  << std::endl;
                        child_id_global = next_level_vec_offset + PointiToId<N>(child_pt, max_dims_[cur_level+1]);
//                        std::cout << "child_id_global = " << child_id_global << std::endl;
                        child_global_ids.push_back(child_id_global);
                        //std::cout << "child_id_global = " << child_id_global << std::endl;
                        assert(tree_ptr_vec_[child_id_global] != nullptr);
                        if(tree_ptr_vec_[child_id_global]->mixed_state_) {
                            all_the_same = false;
                            //break;
                        }
                        if(is_occ == -1) {
                            is_occ = tree_ptr_vec_[child_id_global]->occ_;
                        } else {
                            if(is_occ != tree_ptr_vec_[child_id_global]->occ_) {
                                all_the_same = false;
                                //break;
                            }
                        }
                    }

                    auto new_tree_node = std::make_shared<TreeNodeNew<N> >(nullptr);

                    Pointi<N> pt = IdToPointi<N>(id, max_dims_[cur_level]);

//                    new_tree_node->base_pt_ = tree_ptr_vec_[next_level_vec_offset + id*pow_2_[N]]->base_pt_/pow_2_[N]; // Pointi<N>base
                    new_tree_node->base_pt_ = tree_ptr_vec_[child_global_ids.front()]->base_pt_; // Pointi<N>

                    new_tree_node->occ_ = tree_ptr_vec_[child_global_ids.front()]->occ_;

                    new_tree_node->mixed_state_ = !all_the_same;

                    new_tree_node->depth_ = cur_level;

//                    std::cout << "new node: " << new_tree_node
//                              << ", id = " << cur_level_vec_offset + id
//                              << "，depth = " << new_tree_node->depth_
//                              << ", mixed = " << new_tree_node->mixed_state_
//                              << ", occ = " << new_tree_node->occ_
//                              << ", base_pt = " << new_tree_node->base_pt_
//                              << std::endl;
//
//                    for(const auto& child_id_global : child_global_ids) {
//                        const auto& child_node = tree_ptr_vec_[child_id_global];
//                        std::cout << "child node: " << child_node
//                                  << ", id = " << child_id_global
//                                  << "，depth = " << child_node->depth_
//                                  << ", mixed = " << child_node->mixed_state_
//                                  << ", occ = " << child_node->occ_
//                                  << ", base_pt = " << child_node->base_pt_
//                                  << std::endl;
//                    }

//                    if(all_the_same) {
//                        for(const auto& child_id_global : child_global_ids) {
//                            assert(tree_ptr_vec_[child_id_global] != nullptr);
//                            tree_ptr_vec_[child_id_global] = nullptr;
//                            //std::cout << "set_to_null = " << child_id_global << std::endl;
//                        }
//                    } else {
//                        for(int i=0; i<child_global_ids.size(); i++) {
//                            const auto& child_id_global = child_global_ids[i];
//                            assert(tree_ptr_vec_[child_id_global] != nullptr);
//                            assert(tree_ptr_vec_[child_id_global]->parent_ == nullptr);
//                            tree_ptr_vec_[child_id_global]->parent_ = new_tree_node;
//                            new_tree_node->children_.push_back(tree_ptr_vec_[child_id_global]);
//                        }
//                    }

                    for(int i=0; i<child_global_ids.size(); i++) {
                        const auto& child_id_global = child_global_ids[i];
                        assert(tree_ptr_vec_[child_id_global] != nullptr);
                        assert(tree_ptr_vec_[child_id_global]->parent_ == nullptr);
                        tree_ptr_vec_[child_id_global]->parent_ = new_tree_node;
                        new_tree_node->children_.push_back(tree_ptr_vec_[child_id_global]);
                    }

                    tree_ptr_vec_[cur_level_vec_offset + id] = new_tree_node;

                    //std::cout << "non_null_ptr node id = " << cur_level_vec_offset + id << std::endl;

                    if(cur_level == 0) {
                        root_ = new_tree_node;
                    }
                }
                //break;
                for(int node_id=cur_level_vec_offset; node_id<next_level_vec_offset; node_id++) {
                    if(tree_ptr_vec_[node_id] == nullptr) {
                        //std::cout << "node id " << node_id << " = nullptr" << std::endl;
                        assert(tree_ptr_vec_[node_id] != nullptr);
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

            std::vector<TreeNodeNewPtr<N> > free_leaf_nodes = getAllPassableLeafNodes();
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
        void setBlockPtrForNode(const TreeNodeNewPtr<N>& node, const BlockPtrRaw<N>& block_ptr) {
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

        TreeNodeNewPtrs<N> getAllPassableLeafNodes() const {
            //std::cout << "-- " << __FUNCTION__ << std::endl;
            TreeNodeNewPtrs<N> nodes = { root_ }, next_nodes, retv;
            int dp = 0;
            while (!nodes.empty()) {
                //std::cout << " depth = " << dp << ": " << std::endl;
                for(int i=0; i<nodes.size(); i++) {
                    assert(nodes[i]->depth_ == dp);
                    if(nodes[i]->mixed_state_) {
                        if(nodes[i]->children_.size() != pow_2_[N]) {
                            std::cout << "new node: " << nodes[i]
                                      << "，depth = " << nodes[i]->depth_
                                      << ", mixed = " << nodes[i]->mixed_state_
                                      << ", occ = " << nodes[i]->occ_
                                      << ", base_pt = " << nodes[i]->base_pt_
                                      << std::endl;
                        }
                        assert(nodes[i]->children_.size() == pow_2_[N]);
                    }
                    if(!nodes[i]->mixed_state_ && !nodes[i]->occ_) {
                        retv.push_back(nodes[i]);
                    }
                    if(nodes[i]->mixed_state_) {
                        for (int j = 0; j < pow_2_[N]; j++) {
                            if (nodes[i]->children_[j] != nullptr) {
                                next_nodes.push_back(nodes[i]->children_[j]);
                            }
                        }
                    }
                    //std::cout << std::endl;
                }
                nodes.clear();
                std::swap(nodes, next_nodes);
                dp ++;
            }
            //std::cout << "finish " << __FUNCTION__ << std::endl;
            return retv;
        }

        // find the leaf node that contain current pt, and return its occ
        bool isOccupied(const Pointi<N>& pt) const {
            Id id = PointiToId(pt, max_dims_[max_depth_]);
            //std::cout << "*pt = " << pt << std::endl;
            for(int dp=max_depth_; dp>=0; dp--) {
                const int& id_in_level = PtToTreeVecId(pt, dp);
                //std::cout << "dp = " << dp << ", gid = " << id_in_level <<" | ";
                if(tree_ptr_vec_[id_in_level] != nullptr) {
                    if(!tree_ptr_vec_[id_in_level]->mixed_state_) {
                        //std::cout << std::endl;
                        return tree_ptr_vec_[id_in_level]->occ_;
                    }
                }
            }
            //std::cout << std::endl;
            std::cout << "find no leaf node, shouldn't reach here" << std::endl;
            assert(0);
            return true;
        }

        void setOccupiedState(const Pointi<N>& pt, bool is_occupied) {
            if(getInternalOccState(pt) == is_occupied) {
                //std::cout << "pt " << pt << " have the state in internal map, do not update" << std::endl;
                return;
            }
            setInternalOccState(pt, is_occupied);

            Id id = PointiToId(pt, max_dims_[max_depth_]);
            if(tree_ptr_vec_[id + level_offset_[max_depth_]]->occ_ == is_occupied) {
//                std::cout << "pt " << pt << " have the state " << tree_ptr_vec_[id + level_offset_[max_depth_]]->occ_
//                <<  " in SBT tree, do not update" << std::endl;
                return ;
            }
            if(updated_ids_.find(id) == updated_ids_.end()) {
                updated_pts_.push_back(pt);
                updated_ids_.insert(id);
            } else {
               //std::cout << "pt " << pt << " already in update_pts, do not update" << std::endl;
            }

//            if(!is_occupied) {
//                std::cout << "set " << pt << " to occ = " << is_occupied << std::endl;
//            }
            assert(isOccupied(pt) != is_occupied); // debug only, may cause increases in time cost
            assert(tree_ptr_vec_[id + level_offset_[max_depth_]] != nullptr);
            assert(tree_ptr_vec_[id + level_offset_[max_depth_]]->mixed_state_ == false);
            assert(tree_ptr_vec_[id + level_offset_[max_depth_]]->occ_ != is_occupied);
            tree_ptr_vec_[id + level_offset_[max_depth_]]->occ_ = is_occupied;
        }

        void globalRecursiveUpdate() {
            Pointis<N> cur_pts, next_pts;
            std::set<Id> cur_ids, next_ids;
            Pointi<N> new_pt, parent_pt;
            //std::cout << "updated_pts_ = ";
            for(const auto& pt : updated_pts_) {
                //std::cout << pt << " ";
                new_pt = pt/2;
                Id id = PointiToId<N>(new_pt, max_dims_[max_depth_-1]);
                if(cur_ids.find(id) == cur_ids.end()) {
                    cur_pts.push_back(new_pt);
                    cur_ids.insert(id);
                }
            }
            //std::cout << std::endl;
            std::vector<TreeNodeNewPtr<N> > new_passable_nodes, old_passable_nodes;

            int child_id_global;
            Id cur_id, next_id, global_cur_id;
            for(int lv=max_depth_-1; lv >= 0; lv--) {
                int cur_level_vec_offset = level_offset_[lv];
                int next_level_vec_offset = level_offset_[lv+1];
                next_ids.clear();
                next_pts.clear();
                for(const auto& cur_pt : cur_pts) {
                    bool all_the_same = true;
                    int is_occ = -1;
                    cur_id = PointiToId<N>(cur_pt, max_dims_[lv]);
                    global_cur_id = level_offset_[lv] + cur_id;
                    assert(tree_ptr_vec_[global_cur_id] != nullptr);

                    std::vector<Id> child_global_ids;
                    for(const auto& child_offset : flag_pts_) {
                        //std::cout << "child_id = " << child_id << std::endl;
                        Pointi<N> child_pt = cur_pt*2 + child_offset;
//                        std::cout << "child_pt = " << child_pt << std::endl;
//                        std::cout << "child pt local id = " << PointiToId<N>(child_pt, max_dims_[cur_level+1])  << std::endl;
                        child_id_global = next_level_vec_offset + PointiToId<N>(child_pt, max_dims_[lv+1]);
//                        std::cout << "child_id_global = " << child_id_global << std::endl;
                        child_global_ids.push_back(child_id_global);
                        //std::cout << "child_id_global = " << child_id_global << std::endl;
                        assert(tree_ptr_vec_[child_id_global] != nullptr);
                        if(tree_ptr_vec_[child_id_global]->mixed_state_) {
                            all_the_same = false;
                            //break;
                        }
                        if(is_occ == -1) {
                            is_occ = tree_ptr_vec_[child_id_global]->occ_;
                        } else {
                            if(is_occ != tree_ptr_vec_[child_id_global]->occ_) {
                                all_the_same = false;
                                //break;
                            }
                        }
                    }

                    bool parent_need_update = false; // if current node is updated, parent will need update
                    // if the node's mixed state is changed, its parent may need update
                    if(all_the_same == tree_ptr_vec_[global_cur_id]->mixed_state_) {
                        parent_need_update = true;
                    } else if (all_the_same == true && tree_ptr_vec_[global_cur_id]->mixed_state_ == false) {
                        // if the node is not mixed in previous and now, but its occ state changed, its parent may need update
                        if(is_occ != tree_ptr_vec_[global_cur_id]->occ_) {
                            parent_need_update = true;
                        }
                    }

                    // if previous is unpassable or mixed, now is passable, add to new leaf nodes
                    if(all_the_same && is_occ == false) {
                        if(tree_ptr_vec_[global_cur_id]->mixed_state_ == true || tree_ptr_vec_[global_cur_id]->occ_ == true) {
                            // limit minimum size of blocks
                            if(tree_ptr_vec_[global_cur_id]->depth_ < max_depth_ - min_block_depth_width_) {
                                new_passable_nodes.push_back(tree_ptr_vec_[global_cur_id]);
                            }
                        }
                    }

                    // if previous is passable, now is unpassable or mixed, add to new leaf nodes
                    if(tree_ptr_vec_[global_cur_id]->mixed_state_ == false || tree_ptr_vec_[global_cur_id]->occ_ == false) {
                        if(all_the_same == false || is_occ == true) {
                            old_passable_nodes.push_back(tree_ptr_vec_[global_cur_id]);
                        }
                    }

                    tree_ptr_vec_[global_cur_id]->mixed_state_ = !all_the_same;
                    tree_ptr_vec_[global_cur_id]->occ_ = tree_ptr_vec_[child_global_ids.front()]->occ_;

                    // before reach root, parent may need update
                    //if(lv > 0) { // && parent_need_update) {
                    if(lv > 0 && parent_need_update) {
                            parent_pt = cur_pt / 2;
                        next_id = PointiToId(parent_pt, max_dims_[lv - 1]);
                        if (next_ids.find(next_id) == next_ids.end()) {
                            next_ids.insert(next_id);
                            next_pts.push_back(parent_pt);
                        }
                    }
                }
                std::swap(next_pts, cur_pts);
                std::swap(next_ids, cur_ids);
            }
            updated_pts_.clear();
            updated_ids_.clear();

            // update block ptr
            for(const auto& leaf_node : old_passable_nodes) {
                if(leaf_node->occ_ != false || leaf_node->mixed_state_ != false) {
                    BlockPtrRaw<N> block_ptr = nullptr;
                    setBlockPtrForNode(leaf_node, block_ptr);
                }
                if(leaf_node->mixed_state_ == true) {
                    // set all passable leaf node's block ptr
                    std::vector<TreeNodeNewPtr<N>> buffer = { leaf_node }, next_buffer;
                    while(!buffer.empty()) {
                        next_buffer.clear();
                        for(const auto& temp_node : buffer) {
                            // if child node too small, no need to update block ptr
                            if(temp_node->depth_ + 1 >= max_depth_ - min_block_depth_width_) {
                                continue;
                            }
                            for(const auto& temp_child : temp_node->children_) {
                                if(temp_child->mixed_state_) {
                                    next_buffer.push_back(temp_child);
                                } else if(!temp_child->occ_) {
                                    assert(temp_child->parent_->mixed_state_ || temp_child->parent_->occ_);
                                    BlockPtrRaw<N> block_ptr = std::make_shared<Block<N> >();
                                    block_ptr->min_ = temp_child->base_pt_;
                                    Pointi<N> offset; offset.setAll(pow_2_[max_depth_-temp_child->depth_]-1);
                                    block_ptr->max_ = temp_child->base_pt_ + offset;
                                    setBlockPtrForNode(temp_child, block_ptr);
                                }
                            }
                        }
                        std::swap(buffer, next_buffer);
                    }
                }
            }
            for(const auto& leaf_node : new_passable_nodes) {
                if(leaf_node->occ_ == false && leaf_node->mixed_state_ == false) {
                    BlockPtrRaw<N> block_ptr = nullptr;
                    if(leaf_node->parent_ != nullptr && leaf_node->parent_->occ_ == false &&
                       leaf_node->parent_->mixed_state_ == false) {
                        continue;
                    }
                    block_ptr = std::make_shared<Block<N> >();
                    block_ptr->min_ = leaf_node->base_pt_;
                    Pointi<N> offset; offset.setAll(pow_2_[max_depth_-leaf_node->depth_]-1);
                    block_ptr->max_ = leaf_node->base_pt_ + offset;
                    setBlockPtrForNode(leaf_node, block_ptr);
                }
            }
        }


        void printTree() const {
            std::cout << "-- " << __FUNCTION__ << std::endl;
            TreeNodeNewPtrs<N> nodes = { root_ }, next_nodes;
            int dp = 0;
            while (!nodes.empty()) {
                std::cout << " depth = " << dp << ": " << std::endl;
                for(int i=0; i<nodes.size(); i++) {
                    assert(nodes[i]->depth_ == dp);
                    if(nodes[i]->depth_ < max_depth_) {
                        assert(nodes[i]->children_.size() == pow_2_[N]);
                    }
                    std::cout << nodes[i] << "(occ:" << nodes[i]->occ_
                              << ", mixed_state:" << nodes[i]->mixed_state_
                              << ", depth:" << nodes[i]->depth_
                              << ", base_pt:" << nodes[i]->base_pt_
                              << ")" << "->";
                    if(!nodes[i]->mixed_state_ && nodes[i]->depth_ < max_depth_) {
                        for(int j=0; j<pow_2_[N]; j++) {
                            if(nodes[i]->children_[j] != nullptr) {
                                std::cout << nodes[i]->children_[j]
                                          << "(occ:" << nodes[i]->children_[j]->occ_
                                          << ", mixed_state:" << nodes[i]->children_[j]->mixed_state_ << ") / ";
                                next_nodes.push_back(nodes[i]->children_[j]);
                            }
                        }
                    }
                    std::cout << std::endl;
                }
                nodes.clear();
                std::swap(nodes, next_nodes);
                dp ++;
            }
        }




        IS_OCCUPIED_FUNC <N> isoc_; // notice, setOccupiedState will not change it,
        // so it is wrong after call setOccupiedState after initialize

        DimensionLength *dim_;

        TreeNodeNewPtr <N> root_ = nullptr;

        int max_depth_ = 0;

        int min_block_depth_width_ = 1; // the minimum block width is pow(2, min_block_depth_width_)

        std::vector<int> pow_2_; // precomputation of pow(2, x)

        Pointis <N> flag_pts_; // precomputation of all flag points

        //bool initialized_ = false; // enable update block ptr only after initialized

        TreeNodeNewPtrs<N> tree_ptr_vec_; // size = 1 + pow(2, N) + pow(2, 2N) +...

        std::vector<int> level_offset_; // offset to visit level N's tree node in tree_ptr_vec_

        std::vector<DimensionLength*> max_dims_;

        std::vector<Pointi<N>> updated_pts_; // record what pt was update during update phase, clear after update
        std::set<Id> updated_ids_; // avoid repeat of pt cause extra calculation

    };


    // store all passable block
    // if a block (tree node)'s all children node is nullptr, it is passable
    // otherwise, some part of it are passable and others are occupied,
    // those passable are non-null, unpassable are null
    template<Dimension N>
    class SpaceBinaryTreeAnyDimension : public SpaceBinaryTree<N> {
    public:

        SpaceBinaryTreeAnyDimension(const IS_OCCUPIED_FUNC<N>& isoc, DimensionLength* dim, int min_block_depth_width = 1)
                : SpaceBinaryTree<N>(isoc, dim, min_block_depth_width) {
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

        virtual void setInternalBlockPtr(const Pointi<N>& pt, const BlockPtrRaw<N>& block_ptr) override {
            if(isOutOfBoundary(pt, this->dim_)) { return; }
            block_ptr_map_[PointiToId(pt, this->dim_)] = block_ptr;
        }

        virtual const BlockPtrRaw<N>& getInternalBlockPtr(const Pointi<N>& pt) const override {
            if(isOutOfBoundary(pt, this->dim_)) { return nullptr; }
            return block_ptr_map_[PointiToId(pt, this->dim_)];
        }

        virtual void clearInternalBlockPtr() {
            block_ptr_map_ = BlockPtrsRaw<N>(getTotalIndexOfSpace<N>(this->dim_), nullptr);
        }

        BlockPtrsRaw<N> block_ptr_map_; // save all grid's block ptr need lots space, but reduce time cost

        std::vector<bool> occ_map_; // save all grid's state need lots space, but reduce time cost

    };

}
#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_H
