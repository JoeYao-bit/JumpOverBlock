//
// Created by yaozhuo on 7/22/25.
//

#ifndef JUMPOVERBLOCK_SPACE_BINARY_TREE_H
#define JUMPOVERBLOCK_SPACE_BINARY_TREE_H

#include "common.h"
#include "freeNav-base/basic_elements/misc.h"

namespace freeNav::JOB {

    template<Dimension N>
    class SpaceBinaryTree {
    public:

        SpaceBinaryTree(const IS_OCCUPIED_FUNC <N> &isoc, DimensionLength *dim, int min_block_depth_width = 0)
                : isoc_(isoc), dim_(dim), min_block_depth_width_(min_block_depth_width) {

            //std::cout << "min_block_depth_width = " << min_block_depth_width_ << std::endl;
            all_1_pt_.setAll(1);

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
//            std::cout << "max_depth = " << max_depth_ << std::endl;
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
//            std::cout << "size_of_vec = " << size_of_vec << std::endl;
//            std::cout << " level_offset_ = ";
//            for(int d=0; d<=max_depth_; d++) {
//                std::cout << level_offset_[d] << " ";
//            }
//            std::cout << std::endl;

            tree_ptr_vec_.resize(size_of_vec, nullptr);

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
            // construct local update isoc
            auto is_occupied_temp = [&](const Pointi<N> & pt) -> bool {
                return getInternalOccState(pt);
            };
            isoc_dynamic_ = is_occupied_temp;
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
            mergePassableBlocksViaDecisionTree();
        }

        virtual void setInternalOccState(const Pointi<N>& pt, bool occ_state) = 0;

        virtual bool getInternalOccState(const Pointi<N>& pt) const = 0;

        virtual void setInternalBlockPtr(const Pointi<N>& pt, const BlockPtrRaw<N>& block_ptr) = 0;

        virtual const BlockPtrRaw<N>& getInternalBlockPtr(const Pointi<N>& pt) const = 0;

        virtual void clearInternalBlockPtr() = 0;

        void initBlockPtrMap() {
            //std::cout << "start " << __FUNCTION__ << std::endl;
            clearInternalBlockPtr();

            std::vector<TreeNodeNewPtr<N> > free_leaf_nodes = getAllPassableLeafNodes();
            for(const auto& leaf_node : free_leaf_nodes) {
                if(leaf_node->depth_ > max_depth_ - min_block_depth_width_) { continue; } // limit minimum size of blocks
                //std::cout << "leaf_node->depth_ = " << leaf_node->depth_ << std::endl;
                BlockPtrRaw<N> block_ptr = std::make_shared<BlockRaw<N> >();
                block_ptr->min_ = leaf_node->base_pt_;
                Pointi<N> offset; offset.setAll(pow_2_[max_depth_-leaf_node->depth_]-1);
                block_ptr->max_ = leaf_node->base_pt_ + offset;
                block_ptr->tree_node_ = leaf_node;
                //leaf_node->block_ptr_ = block_ptr;
                setBlockPtrForNode(leaf_node, block_ptr);
            }
            //std::cout << "finish " << __FUNCTION__ << std::endl;
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
//            //std::cout << "*pt = " << pt << std::endl;
//            for(int dp=max_depth_; dp>=0; dp--) {
//                const int& id_in_level = PtToTreeVecId(pt, dp);
//                //std::cout << "dp = " << dp << ", gid = " << id_in_level <<" | ";
//                if(tree_ptr_vec_[id_in_level] != nullptr) {
//                    if(!tree_ptr_vec_[id_in_level]->mixed_state_) {
//                        //std::cout << std::endl;
//                        return tree_ptr_vec_[id_in_level]->occ_;
//                    }
//                }
//            }
//            //std::cout << std::endl;
//            std::cout << "find no leaf node, shouldn't reach here" << std::endl;
//            assert(0);
            return tree_ptr_vec_[id + level_offset_[max_depth_]]->occ_;
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
                            if(tree_ptr_vec_[global_cur_id]->depth_ <= max_depth_ - min_block_depth_width_) {
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
                            if(temp_node->depth_ + 1 > max_depth_ - min_block_depth_width_) {
                                continue;
                            }
                            for(const auto& temp_child : temp_node->children_) {
                                if(temp_child->mixed_state_) {
                                    next_buffer.push_back(temp_child);
                                } else if(!temp_child->occ_) {
                                    assert(temp_child->parent_->mixed_state_ || temp_child->parent_->occ_);
                                    BlockPtrRaw<N> block_ptr = std::make_shared<BlockRaw<N> >();
                                    block_ptr->min_ = temp_child->base_pt_;
                                    Pointi<N> offset; offset.setAll(pow_2_[max_depth_-temp_child->depth_]-1);
                                    block_ptr->max_ = temp_child->base_pt_ + offset;
                                    block_ptr->tree_node_ = temp_child;
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
                    block_ptr = std::make_shared<BlockRaw<N> >();
                    block_ptr->min_ = leaf_node->base_pt_;
                    Pointi<N> offset; offset.setAll(pow_2_[max_depth_-leaf_node->depth_]-1);
                    block_ptr->max_ = leaf_node->base_pt_ + offset;
                    block_ptr->tree_node_ = leaf_node;
                    setBlockPtrForNode(leaf_node, block_ptr);
                }
            }

            mergePassableBlocksViaDecisionTree();

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
//                    std::cout << nodes[i] << "(occ:" << nodes[i]->occ_
//                              << ", mixed_state:" << nodes[i]->mixed_state_
//                              << ", depth:" << nodes[i]->depth_
//                              << ", base_pt:" << nodes[i]->base_pt_
//                              << ")" << "->";
                    if(!nodes[i]->mixed_state_ && nodes[i]->depth_ < max_depth_) {
                        for(int j=0; j<pow_2_[N]; j++) {
                            if(nodes[i]->children_[j] != nullptr) {
//                                std::cout << nodes[i]->children_[j]
//                                          << "(occ:" << nodes[i]->children_[j]->occ_
//                                          << ", mixed_state:" << nodes[i]->children_[j]->mixed_state_ << ") / ";
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

        struct MergedBlock;
        typedef std::shared_ptr<MergedBlock> MergedBlockPtr;
        typedef std::vector<MergedBlockPtr> MergedBlockPtrs;

        struct MergedBlock {
            Pointi<N> min_pt_;
            Pointi<N> max_pt_;
            BlockPtrsRaw<N> block_ptrs_; // leaf node of SBT in current block

            MergedBlockPtr parent_;
            MergedBlockPtrs children_;
        };

        struct BlockCompare {
            bool operator()(const BlockPtrRaw<N>& a, const BlockPtrRaw<N>& b) const {
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

        std::set<BlockPtrRaw<N> >
                tryExpandBlockInDirection(int dim,
                                          const MergedBlockPtr& block_node,
                                          const std::map<BlockPtrRaw<N>, bool, BlockCompare>& leave_merged_map) const {

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
            BlockPtrRaw<N> neighbor_block_ptr = nullptr;
            int expected_num_of_neighbor_block = 0;
            std::set<BlockPtrRaw<N> > neighbor_block_ptrs;
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
                BlockPtrRaw<N> temp_block_ptr = getInternalBlockPtr(new_plain_pt);
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
                    expected_num_of_neighbor_block = (total_plain_index + 1) / area_of_neighbor_node;
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
            MSTimer mst;
            merged_block_ptrs_.clear();
            // big block start, small block end
            const auto& all_leaves = getAllPassableLeafNodes();
            std::map<BlockPtrRaw<N>, bool, BlockCompare> leave_merged_map;
            BlockPtrsRaw<N> all_blocks;
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
                MergedBlockPtr root = std::make_shared<MergedBlock>();
                root->min_pt_ = block_ptr->min_;
                root->max_pt_ = block_ptr->max_;
                root->block_ptrs_ = {block_ptr};
                //std::cout << "root block: min_pt_ = " << root->min_pt_ << " / max_pt = " << root->max_pt_  << std::endl;

                MergedBlockPtr largest_merged_block_ptr = root;
                int largest_size = getTotalIndexOfSpace(largest_merged_block_ptr->max_pt_ - largest_merged_block_ptr->min_pt_);

                MergedBlockPtrs buffer = {root}, next_buffer;

                leave_merged_map[block_ptr] = true;

                while(!buffer.empty()) {
                    next_buffer.clear();
                    for(const auto& block_node : buffer) {
                        // expand current block until cannot expand
                        for (int dim = 0; dim < 2 * N; dim++) {
                            // if there are required number of tree node in this direction
                            // we say we found a legal expansion
                            auto neighbor_block_ptrs = tryExpandBlockInDirection(dim, block_node, leave_merged_map);
                            if(!neighbor_block_ptrs.empty()) {
                                //std::cout << "expand in dir " << dim << " success" << std::endl;
                                MergedBlockPtr new_block_node = std::make_shared<MergedBlock>();
                                new_block_node->min_pt_ = block_node->min_pt_;
                                new_block_node->max_pt_ = block_node->max_pt_;
                                new_block_node->parent_ = block_node;
                                new_block_node->block_ptrs_ = block_node->block_ptrs_;
                                int expand_dist = pow_2_[max_depth_ - (*neighbor_block_ptrs.begin())->tree_node_->depth_];
                                // update block size
                                if(dim % 2 == 1) {
                                    new_block_node->max_pt_[dim/2] = new_block_node->max_pt_[dim/2] + expand_dist;
                                } else {
                                    new_block_node->min_pt_[dim/2] = new_block_node->min_pt_[dim/2] - expand_dist;
                                }
                                for(const auto& neighbor_block_ptr : neighbor_block_ptrs) {
                                    new_block_node->block_ptrs_.push_back(neighbor_block_ptr);
                                }
                                block_node->children_.push_back(new_block_node);
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
            std::cout << __FUNCTION__ << " take " << mst.elapsed() << " ms" << std::endl;
        }

        bool lineCrossObstacleRaw(const Pointi<N>& pt1, const Pointi<N>& pt2, IS_OCCUPIED_FUNC<N> is_occupied) {
            if(pt1 == pt2) return true;
            Line<N> line(pt1, pt2);
            int check_step = line.step;
            Pointi<N> pt;
            //std::cout << __FUNCTION__ << std::endl;
            for(int i=1; i<check_step; i++) {
                pt = line.GetPoint(i);
                raw_visited_pt_count_ ++;
                //std::cout << "raw_visited_pt_count_ = " << raw_visited_pt_count_ << std::endl;
                if(is_occupied(pt)) {
                    return true;
                }
            }
            return false;
        }

//        bool lineCrossObstacleSBT(const Pointi<N>& pt1, const Pointi<N>& pt2, const IS_OCCUPIED_FUNC<N>& is_occupied,
//                                  Pointis<N>& visited_pt,
//                                  int& count_of_block
//        ) {
//            //if(isOutOfBoundary(pt1, dim_) || isOutOfBoundary(pt2, dim_)) { return true; }
//            if(pt1 == pt2) return true;
//            //visited_pt.clear();
//            //count_of_block = 0;
//            Line<N> line(pt1, pt2);
//            int check_step = line.step;
//            Pointi<N> pt;
//            Id id;
//            int jump_step = 0;
//            for(int i=1; i<check_step; i++) {
//                pt = line.GetPoint(i);
//                SBT_visited_pt_count_ ++;
//                if(is_occupied(pt)) { return true; }
//                const auto& block_ptr = getInternalBlockPtr(pt);
//                // if in block, jump over current block
//                if(block_ptr != nullptr) {
//                    jump_step = findExitPointOfBlock(line, pt, i, static_cast<BlockPtr<N>>(block_ptr));
//                    //std::cout << " jump step " << jump_step << std::endl;
//                    i = i + jump_step;
//                    count_of_block ++;
//                }
//            }
//            return false;
//        }

        bool lineCrossObstacleSBT(const Pointi<N>& pt1, const Pointi<N>& pt2, const IS_OCCUPIED_FUNC<N>& is_occupied,
                                  Pointis<N>& visited_pt,
                                  int& count_of_block
        ) {
            //if(isOutOfBoundary(pt1, dim_) || isOutOfBoundary(pt2, dim_)) { return true; }
            if(pt1 == pt2) return true;
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

//        float getObstacleDensity() {
//            // count passable children grid
//            float count = 0;
//            Id total_index = getTotalIndexOfSpace<N>(max_dims_[max_depth_]);
//            for(int i=0; i<total_index; i++) {
//                const auto& tree_node_ptr = tree_ptr_vec_[i + level_offset_[max_depth_]];
//                assert(tree_node_ptr != nullptr);
//                if(tree_node_ptr->occ_ == false && tree_node_ptr->mixed_state_ == false) {
//                    count ++;
//                }
//            }
//            return count / getTotalIndexOfSpace<N>(dim_);
//        }

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

        IS_OCCUPIED_FUNC<N> isoc_dynamic_;  // notice, set state will change it

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

        MergedBlockPtrs merged_block_ptrs_;

        int raw_visited_pt_count_ = 0, SBT_visited_pt_count_ = 0;// for debug

    };

    template<Dimension N>
    using SpaceBinaryTreePtr = std::shared_ptr<SpaceBinaryTree<N> >;

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


    class SpaceBinaryTree2D : public SpaceBinaryTree<2> {
    public:

        SpaceBinaryTree2D(const IS_OCCUPIED_FUNC<2>& isoc, DimensionLength* dim, int min_block_depth_width = 4)
                : SpaceBinaryTree<2>(isoc, dim, min_block_depth_width) {
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

        virtual void setInternalBlockPtr(const Pointi<2>& pt, const BlockPtrRaw<2>& block_ptr) override {
            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1]) { return; }
            block_ptr_map_[pt[0] + pt[1]*dim_[0]] = block_ptr;
        }

        virtual const BlockPtrRaw<2>& getInternalBlockPtr(const Pointi<2>& pt) const override {
            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1]) { return nullptr; }
            return block_ptr_map_[pt[0] + pt[1]*dim_[0]];
        }

        virtual void clearInternalBlockPtr() {
            block_ptr_map_ = BlockPtrsRaw<2>(getTotalIndexOfSpace<2>(dim_), nullptr);
        }

        BlockPtrsRaw<2> block_ptr_map_; // save all grid's block ptr need lots space, but reduce time cost

        std::vector<bool> occ_map_; // save all grid's state need lots space, but reduce time cost


    };


    class SpaceBinaryTree3D : public SpaceBinaryTree<3> {
    public:

        SpaceBinaryTree3D(const IS_OCCUPIED_FUNC<3>& isoc, DimensionLength* dim, int min_block_depth_width = 3)
                : SpaceBinaryTree<3>(isoc, dim, min_block_depth_width) {
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

        virtual void setInternalBlockPtr(const Pointi<3>& pt, const BlockPtrRaw<3>& block_ptr) override {
//            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1] || pt[2] < 0 || pt[2] >= dim_[2]) { return ; }
            block_ptr_map_[pt[0] + pt[1]*dim_[0] + pt[2]*dim_[0]*dim_[1]] = block_ptr;
            //if(isOutOfBoundary(pt, this->dim_)) { return ; }
            //block_ptr_map_[PointiToId(pt, dim_)] = block_ptr;
        }

        virtual const BlockPtrRaw<3>& getInternalBlockPtr(const Pointi<3>& pt) const override {
//            if(pt[0] < 0 || pt[0] >= dim_[0] || pt[1] < 0 || pt[1] >= dim_[1] || pt[2] < 0 || pt[2] >= dim_[2]) { return nullptr; }
            return block_ptr_map_[pt[0] + pt[1]*dim_[0] + pt[2]*dim_[0]*dim_[1]];
//            if(isOutOfBoundary(pt, this->dim_)) { return nullptr; }
//            return block_ptr_map_[PointiToId(pt, dim_)];
        }

        virtual void clearInternalBlockPtr() {
            block_ptr_map_ = BlockPtrsRaw<3>(getTotalIndexOfSpace<3>(dim_), nullptr);
        }

        BlockPtrsRaw<3> block_ptr_map_; // save all grid's block ptr need lots space, but reduce time cost

        std::vector<bool> occ_map_; // save all grid's state need lots space, but reduce time cost


    };

}
#endif //JUMPOVERBLOCK_SPACE_BINARY_TREE_H
