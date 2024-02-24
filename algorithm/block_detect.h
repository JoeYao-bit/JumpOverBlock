//
// Created by yaozhuo on 2023/4/18.
//

#ifndef FREENAV_BLOCK_DETECT_H
#define FREENAV_BLOCK_DETECT_H

#include "../freeNav-base/basic_elements/map_down_sampler.h"
#include "../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/dependencies/self_sorted_queue.h"
#include "../freeNav-base/basic_elements/surface_process.h"
#include <fstream>
#include "octomap/octomap.h"

namespace freeNav::JOB {

    template <Dimension N>
    class BlockDetectorInterface;

    template <Dimension N>
    using BlockDetectorInterfacePtr = std::shared_ptr<BlockDetectorInterface<N> >;

    template <Dimension N>
    class BlockDetector;

    template <Dimension N>
    using BlockDetectorPtr = std::shared_ptr<BlockDetector<N> >;

    class BlockDetectorOctoMap;

    using BlockDetectorOctoMapPtr = std::shared_ptr<BlockDetectorOctoMap>;

    template <Dimension N>
    struct Block;

    template <Dimension N>
    using BlockPtr = std::shared_ptr<Block<N> >;

    template <Dimension N>
    using BlockPtrs = std::vector<BlockPtr<N> >;

    template <Dimension N>
    struct Block{
        //Pointis<N> corner_pts_; // 2^N corner points
        //Pointi<N> center_pt_;

        Pointi<N> min_, max_; // the minimum and the maximum point in all corner points

        void updateDimension() {
            Pointi<N> dim_offset = max_ - min_;
            for(Dimension i=0; i<N; i++) {
                dimen_[i] = dim_offset[i] + 1;
            }
        }

        bool PointiInBlock(const Pointi<N>& pt) {
            for(Dimension dim=0; dim<N; dim++) {
                if(pt[dim] < min_[dim] || pt[dim] > max_[dim]) { return false; }
            }
            return true;
        }

        DimensionLength dimen_[N]; // not original data, update with max_ and min_

        std::vector<bool> is_expandable_ = std::vector<bool>(2*N, true);
    };



    template <Dimension N>
    class BlockDetectorInterface {
    public:

        explicit BlockDetectorInterface(DimensionLength* dimension_info,
                                        const IS_OCCUPIED_FUNC<N>& is_occupied,
                                        const std::string& file_path = "",
                                        PathLen minimum_block_width = 10,
                                        bool force_update = false) {
            dimension_info_ = dimension_info;
            is_occupied_ = is_occupied;
            min_block_width_ = minimum_block_width;
            file_path_ = file_path;
        }

        virtual void detectBlock() = 0;

        virtual bool deserialize(const std::string& file_path) {
            if(file_path == "") return false;
            std::ifstream vis_bin(file_path, std::ios_base::in | std::ios_base::binary);
            if(vis_bin.fail()) return false;
            PathLen temp_min_block_width;
            vis_bin.read((char*)&temp_min_block_width, sizeof (PathLen));
            if(temp_min_block_width != min_block_width_) {
                std::cout << " min_block_width in file = " << temp_min_block_width << " != " << min_block_width_ << std::endl;
                return false;
            }
            Id pt_id; int count = 0;
            Pointi<N> min_pt, max_pt;
            this->block_ptr_map_ = BlockPtrs<N>(getTotalIndexOfSpace<N>(this->dimension_info_), nullptr);
            while(vis_bin.read((char*)&pt_id, sizeof (Id))) {
                if(count % 2 == 0) {
                    min_pt = IdToPointi<N>(pt_id, this->dimension_info_);
                } else {
                    max_pt = IdToPointi<N>(pt_id, this->dimension_info_);
                    BlockPtr<N> block_ptr = std::make_shared<Block<N> >();
                    block_ptr->min_ = min_pt;
                    block_ptr->max_ = max_pt;
                    //std::cout << " min/max pt " << min_pt << " " << max_pt << std::endl;
                    block_ptr->updateDimension();
                    this->all_block_ptrs_.push_back(block_ptr);
                    // set block_ptr_map
                    DimensionLength* block_dim = block_ptr->dimen_;
                    Id total_index = getTotalIndexOfSpace<N>(block_dim);
                    Pointi<N> block_offset, temp_pt;
                    Id temp_id;
                    for(Id id=0; id<total_index; id++) {
                        block_offset = IdToPointi<N>(id, block_dim);
                        temp_pt = min_pt + block_offset;
                        temp_id = PointiToId(temp_pt, this->dimension_info_);
                        if(this->block_ptr_map_[temp_id] != nullptr) {
                            std::cout << " block " << min_pt << "->" << max_pt << " overlap with other block"
                                      << this->block_ptr_map_[temp_id]->min_ << "->" << this->block_ptr_map_[temp_id]->max_ << std::endl;
                            vis_bin.close();
                            return false;
                        }
                        this->block_ptr_map_[temp_id] = block_ptr;
                    }
                }
                count ++;
            }
            vis_bin.close();
            this->all_block_ptrs_.shrink_to_fit();
            return !this->all_block_ptrs_.empty();
        }

        virtual bool serialize(const std::string& file_path) {
            if(file_path == "") return false;
            std::ofstream vis_bin(file_path, std::ios_base::out|std::ios_base::binary|std::ios_base::trunc);
            if(vis_bin.fail()) return false;
            vis_bin.write((char *) &min_block_width_, sizeof(PathLen));
            Id min_pt_id, max_pt_id;
            for(int i=0; i<this->all_block_ptrs_.size(); i++) {
                min_pt_id = PointiToId(this->all_block_ptrs_[i]->min_, this->dimension_info_);
                max_pt_id = PointiToId(this->all_block_ptrs_[i]->max_, this->dimension_info_);
                //std::cout << " min/max pt " << all_block_ptrs_[i]->min_ << " " << all_block_ptrs_[i]->max_ << std::endl;
                vis_bin.write((char *) &min_pt_id, sizeof(Id));
                vis_bin.write((char *) &max_pt_id, sizeof(Id));
            }
            vis_bin.close();
            std::cout << "-- haha save " << this->all_block_ptrs_.size() << " blocks" << std::endl;
            return true;
        }

        IS_OCCUPIED_FUNC<N> is_occupied_;

        DimensionLength *dimension_info_;

        BlockPtrs<N> block_ptr_map_; // store whether the grid in block; if not in block, ptr = nullptr

        BlockPtrs<N> all_block_ptrs_;

        PathLen min_block_width_;// minimum block width, the block may be a square or rectangle

        std::string file_path_;

    };

    template<Dimension N>
    class BlockDetector : public BlockDetectorInterface<N> {
    public:
        explicit BlockDetector(DimensionLength* dimension_info,
                               const IS_OCCUPIED_FUNC<N>& is_occupied,
                               const GridPtrs<N>& corner_grids,
                               PathLen minimum_block_width = 10,
                               const std::string& file_path = "",
                               bool force_update = false) :
                               BlockDetectorInterface<N>(dimension_info, is_occupied, file_path, minimum_block_width, force_update) {
            corner_grids_ = corner_grids;
            determineObstacleRegion();
            if(!force_update && this->deserialize(file_path)) {
                std::cout << "-- load " << this->all_block_ptrs_.size() << " blocks from " << file_path << " success " << std::endl;
            } else {
                detectBlock();
                if(this->serialize(file_path)) {
                    std::cout << "-- save blocks success " << std::endl;
                } else {
                    std::cout << "-- save blocks failed " << std::endl;
                }
            }
        }


        void detectBlock() override {
            std::cout << "-- BlockDetector load blocks from " << this->file_path_ << " failed, try to detect" << std::endl;
            all_direction_local_moves_ = initAllDirectionLocalMoves<N>();
            detectCornerNodes(this->corner_grids_);
            self_sort_heap_ = std::make_shared<AutoSortHeap<PathLen, Id> >(corner_grids_.size());
            waveFront();
            detectLocalMinimals();
            expandFromMinimals();
            std::cout << "-- BlockDetector detect " << this->all_block_ptrs_.size() << " blocks " << std::endl;
        }

    private:

        // detect local minima of distance to nearest node, from all free grid
        // override for acceleration in map with sparse obstacles
        virtual void detectCornerNodes(const GridPtrs<N>& corner_grids) {
            surface_nodes_.clear();
            //std::cout << " surface grids: ";
            for(auto & node_ptr : corner_grids) {
                surface_nodes_.push_back(node_ptr->pt_);
                //std::cout << node_ptr->pt_ << " ";
            }
            //std::cout << std::endl;
        }

        bool isAllExpansionStop(const std::vector<Pointis<N> > & all_direction_grids) {
            for(const auto & each_direction : all_direction_grids) {
                if(!each_direction.empty()) {
                    return false;
                }
            }
            return true;
        }

        Pointis<N> expandCurrentPoint(const Pointi<N>& current_pt, int dim) {
            // expand current point
            Pointis<N> next_expansion;
            Id current_id = PointiToId(current_pt, this->dimension_info_);
            Pointi<N> new_pt;
            Id new_id;
            PathLen new_dist;
            for(const auto& offset : all_direction_local_moves_[dim])
            {
                new_pt = current_pt + offset;
                if(this->is_occupied_(new_pt)) continue;
                new_id = PointiToId(new_pt, this->dimension_info_);
                // stop expansion when hit obstacle, smaller dist to obstacle
                if(dist_map_[new_id] == MAX<PathLen>) {
                    //std::cout << " closet_pt_map_[current_id] " << closet_pt_map_[current_id] << std::endl;
                    new_dist = (new_pt-closet_pt_map_[current_id]).Norm();
                    dist_map_[new_id] = new_dist;
                    closet_pt_map_[new_id] = closet_pt_map_[current_id];
                    obstacle_region_map_[new_id] = obstacle_region_map_[current_id];
                    next_expansion.push_back(new_pt);
                } else {
                    //std::cout << " closet_pt_map_[new_id] " << closet_pt_map_[new_id] << std::endl;
                    new_dist = (new_pt-closet_pt_map_[current_id]).Norm();
                    // considering the order of expansion, allow equal to considering expansion of two nearby corner
                    // with out equal, may cause slit that never visit
                    if(new_dist < dist_map_[new_id]) {
                        dist_map_[new_id] = new_dist;
                        closet_pt_map_[new_id] = closet_pt_map_[current_id];
                        obstacle_region_map_[new_id] = obstacle_region_map_[current_id];
                        next_expansion.push_back(new_pt);
                    }
                }
                // when occurred small dist, mark a local minimal
            }
            return next_expansion;
        }

        // expand dist to nearest node from each tangent node in graph
        // considering limit range of expansion for acceleration ?
        void waveFront() {
            dist_map_ = std::vector<PathLen>(getTotalIndexOfSpace<N>(this->dimension_info_), MAX<PathLen>);
            closet_pt_map_ = std::vector<Pointi<N> >(getTotalIndexOfSpace<N>(this->dimension_info_));
            this->block_ptr_map_ = BlockPtrs<N>(getTotalIndexOfSpace<N>(this->dimension_info_), nullptr);
            // set dist of tangent node to obstacle to 1
            for(const auto& grid : corner_grids_)
            {
                dist_map_[grid->id_] = 0;//(obst - grid->pt_).Norm();
                //std::cout << " obst " << obst << std::endl;
                closet_pt_map_[grid->id_] = grid->pt_;//obst;
                for(const auto& obst : grid->nearby_obst_pts_) {
                    if(isOutOfBoundary(obst, this->dimension_info_)) {
                        obstacle_region_map_[grid->id_] = 0;
                    } else {
                        Id obst_id = PointiToId(obst, this->dimension_info_);
                        obstacle_region_map_[grid->id_] = obstacle_region_map_[obst_id];
                    }
                }
            }
            Pointis<N> next_expansion;
            //for(const auto& tangent_node : tangent_nodes_)
            {
                std::vector<Pointis<N> > all_direction_grids(2 * N, surface_nodes_);//{tangent_node});

                while (!isAllExpansionStop(all_direction_grids)) {
                    // each tangent node expand simultaneously
                    for (Dimension dim = 0; dim < N * 2; dim++) {
                        next_expansion.clear();
                        for (const auto &current_pt : all_direction_grids[dim]) {
                            const auto& current_pt_expansion = expandCurrentPoint(current_pt, dim);
                            next_expansion.insert(next_expansion.end(), current_pt_expansion.begin(),
                                                  current_pt_expansion.end());
                        }
                        std::swap(all_direction_grids[dim], next_expansion);
                    }
                }
            }
        }

        bool isLocalMinimal(const Id& id, const Pointis<N>& nearby_offsets) {
            const PathLen& current_dist = dist_map_[id];
            Pointi<N> current_pt = IdToPointi<N>(id, this->dimension_info_);
            if(this->is_occupied_(current_pt)) { return false; }
            Pointi<N> new_pt;
            Id new_id;
            for(const auto& nearby_offset : nearby_offsets) {
                new_pt = nearby_offset + current_pt;
                if(!isOutOfBoundary(new_pt, this->dimension_info_)) {
                    new_id = PointiToId(new_pt, this->dimension_info_);
                    // near obstacle is not local minimal
                    if(current_dist < dist_map_[new_id] && !this->is_occupied_(new_pt)) {
                        return false;
                    }
                } else {
                    return false;
                }
            }
            return true;
        }

        // detect local minima of distance to nearest node, from all free grid
        void detectLocalMinimals() {
            self_sort_heap_->reset();
            local_minimal_pts_.clear();
            isolated_minimal_pts_.clear();
            int total_count = getTotalIndexOfSpace<N>(this->dimension_info_);
            Pointis<N> nearby_offsets = GetNeightborOffsetGrids<N>();
            for(Id id=0; id<total_count; id++) {
                if(isLocalMinimal(id, nearby_offsets) && dist_map_[id] >= this->min_block_width_ && dist_map_[id] < MAX<PathLen>) {
                    //std::cout << dist_map_[i] << " > " << min_block_width_ << std::endl;
                    local_minimal_pts_.push_back(IdToPointi<N>(id, this->dimension_info_));
                    // as we want pop the maximum value first, we insert the negative value
                    self_sort_heap_->HeapInsert({-dist_map_[id], withIndex<Id>(id, nullptr)});
                }
            }
            // merge close local minimal, feasible but may time consuming
            int half_width = floor(this->min_block_width_/sqrt(N)); // floor to ensure safety
            while(!self_sort_heap_->isEmpty()) {
                auto sort_pair = self_sort_heap_->HeapPopMin();
                Pointi<N> pt = IdToPointi<N>(sort_pair.second.val_, this->dimension_info_);
                bool too_close = false;
                for(const auto& pre_pt : isolated_minimal_pts_) {
                    Pointi<N> abs_offset = toAbs(pre_pt - pt);
                    if(abs_offset.maxDim() <= 2*half_width + 1) { too_close = true; break; }
                }
                if(!too_close)
                {
                    isolated_minimal_pts_.push_back(pt);
                }
            }
        }

        void determineObstacleRegion() {

            detectCornerNodes(this->corner_grids_);
            Id total_index = getTotalIndexOfSpace<N>(this->dimension_info_);
            Pointis<N> nearby_offsets = GetNeightborOffsetGrids<N>();
            obstacle_region_map_.resize(total_index, MAX<Id>);
            Pointi<N> pt;
            Id region_count = 0;

            // expand all obstacle that on map boundary
            for(int dim=0; dim<2*N; dim++) {
                DimensionLength* block_dim = this->dimension_info_;
                DimensionLength plain[N-1];
                for(Dimension i=0; i<dim/2; i++) {
                    plain[i] = block_dim[i];
                }
                for(Dimension i=dim/2+1; i<N; i++) {
                    plain[i-1] = block_dim[i];
                }
                // get the dimension of the N-1 dimension plain
                Id total_plain_index = getTotalIndexOfSpace<N-1>(plain);
                Pointi<N-1> plain_pt;
                Pointi<N> new_plain_pt;
                Id new_plain_pt_id;
                std::vector<Id> new_plain_ids;
                // set the origin of plain
                Pointi<N> origin_of_plain;
                if(dim%2 == 1) {
                    origin_of_plain[dim/2] = this->dimension_info_[dim/2] - 1;
                } else {
                    origin_of_plain[dim/2] = 0;
                }
                for(Id pid=0; pid<total_plain_index; pid++) {
                    plain_pt = IdToPointi<N-1>(pid, plain);
                    new_plain_pt = origin_of_plain.addPlainOffset(plain_pt, dim);
                    new_plain_pt_id = PointiToId(new_plain_pt, this->dimension_info_);
                    if(this->is_occupied_(new_plain_pt)) {
                        if(obstacle_region_map_[new_plain_pt_id] == MAX<Id>) {
                            expandWholeRegion(new_plain_pt, region_count, nearby_offsets);
                        }
                    }
                }
            }
            region_count ++;
            // expand obstacle that didn't cross with boundary
            for(Id id=0; id<total_index; id++) {
                pt = IdToPointi<N>(id, this->dimension_info_);
                if(!this->is_occupied_(pt)) { continue; }
                if(obstacle_region_map_[id] == MAX<Id>) {
                    expandWholeRegion(pt, region_count, nearby_offsets);
                    region_count ++;
                }
            }

        }

        void expandWholeRegion(const Pointi<N>& pt, const Id& region_id, const Pointis<N>& nearby_offsets) {
            Pointis<N> current_set = {pt}, next_set;
            Pointi<N> new_pt; Id new_id;
            while(!current_set.empty()) {
                next_set.clear();
                for(const auto& current_pt : current_set) {
                    for(const auto& offset : nearby_offsets) {
                        new_pt = current_pt + offset;
                        new_id = PointiToId(new_pt, this->dimension_info_);
                        if(isOutOfBoundary(new_pt, this->dimension_info_)) { continue; }
                        if(!this->is_occupied_(new_pt)) { continue; }
                        if(obstacle_region_map_[new_id] == MAX<Id>) {
                            obstacle_region_map_[new_id] = region_id;
                            next_set.push_back(new_pt);
                        } else {
                            continue;
                        }
                    }
                }
                std::swap(current_set, next_set);
            }
        }

        // expand from single local minimal
        BlockPtr<N> expandFromLocalMinimal(const Pointi<N>& center_pt) {
            BlockPtr<N> block_ptr = std::make_shared<Block<N> >();
            //block_ptr->center_pt_ = center_pt;
            int half_width = floor(this->min_block_width_/sqrt(N));//floor(dist_map_[id]/sqrt(N)); // floor to ensure safety

            // set corner point of the cube
            Pointi<N> all_1_pt = GetFloorOrCeilFlag<N>().back();

            block_ptr->min_ = center_pt - all_1_pt*half_width;
            block_ptr->max_ = center_pt + all_1_pt*half_width;
            block_ptr->updateDimension();

            DimensionLength* dims = block_ptr->dimen_;

            Id total_count = getTotalIndexOfSpace<N>(dims);
            Pointi<N> new_pt;
            Id new_id;
            for(Id id=0; id<total_count; id++) {
                new_pt = IdToPointi<N>(id, dims) + block_ptr->min_;
                new_id = PointiToId(new_pt, this->dimension_info_);
                if(this->block_ptr_map_[new_id] != nullptr) {
                    std::cout << " overlap block in " << __FUNCTION__ << std::endl;
                }
                this->block_ptr_map_[new_id] = block_ptr;
            }
            return block_ptr;
        }

        // if is legal, update with new plain block ptr in the ptr map
        bool isLegalToExpandBlockInDirectionOneDirection(const BlockPtr<N>& block_ptr, int dim) {
            DimensionLength* block_dim = block_ptr->dimen_;
            DimensionLength plain[N-1];
            for(Dimension i=0; i<dim/2; i++) {
                plain[i] = block_dim[i];
            }
            for(Dimension i=dim/2+1; i<N; i++) {
                plain[i-1] = block_dim[i];
            }
            // get the dimension of the N-1 dimension plain
            Id total_plain_index = getTotalIndexOfSpace<N-1>(plain);
            Pointi<N-1> plain_pt;
            Pointi<N> new_plain_pt;
            Id new_plain_pt_id;
            std::vector<Id> new_plain_ids;
            // set the origin of plain
            Pointi<N> origin_of_plain = block_ptr->min_;
            if(dim%2 == 1) {
                origin_of_plain[dim/2] = block_ptr->max_[dim/2] + 1;
            } else {
                origin_of_plain[dim/2] -= 1;
            }
            // travel all point in the plain, if all is passable and not intersect with other block, expand the block in the direction
            for(Id id=0; id<total_plain_index; id++) {
                plain_pt = IdToPointi<N-1>(id, plain);
                new_plain_pt = origin_of_plain.addPlainOffset(plain_pt, dim);
                new_plain_pt_id = PointiToId(new_plain_pt, this->dimension_info_);
                if(this->is_occupied_(new_plain_pt) || this->block_ptr_map_[new_plain_pt_id] != nullptr) {
                    block_ptr->is_expandable_[dim] = false; // remember what plain couldn't expand, avoid check again
                    return false;
                }
                new_plain_ids.push_back(new_plain_pt_id);
            }
            // if could be expand in the direction, mark
            for(const auto& new_pt_id : new_plain_ids) {
                if(this->block_ptr_map_[new_pt_id] != nullptr) {
                    std::cout << " overlap block in " << __FUNCTION__ << std::endl;
                }
                this->block_ptr_map_[new_pt_id] = block_ptr;
            }
            if(dim%2 == 1) {
                block_ptr->max_[dim/2] ++;
            } else {
                block_ptr->min_[dim/2] --;
            }
            block_ptr->updateDimension();
            return true;
        }

        // need test
        bool expandBlockOneStep(const BlockPtr<N>& block_ptr) {
            bool is_expanded = false;
            for(Dimension dim=0; dim<2*N; dim++) {
                if(block_ptr->is_expandable_[dim] && isLegalToExpandBlockInDirectionOneDirection(block_ptr, dim)) {
                    is_expanded = true;
                }
            }
            return is_expanded;
        }

        // expand from local minimal points, until reach boundary of the block
        void expandFromMinimals() {
            Pointis<N> corner_offset = GetFloorOrCeilFlag<N>();
            std::cout << std::endl;
            for(const auto& pt : isolated_minimal_pts_) {
                BlockPtr<N> block_ptr = expandFromLocalMinimal(pt);
                this->all_block_ptrs_.push_back(block_ptr);
            }

            // expand plain that still not hit obstacle in it's direction, stop when hit obstacles/another block
            // TODO: what if expand those that far to obstacle first, rather expand all direction simultaneously
            BlockPtrs<N> expandable_block_ptrs = this->all_block_ptrs_;//{ all_block_ptrs_.front() };
            while(1) {
                BlockPtrs<N> next_expandable_block_ptrs;
                for(const auto& block_ptr : expandable_block_ptrs) {
                    if(expandBlockOneStep(block_ptr)) {
                        next_expandable_block_ptrs.push_back(block_ptr);
                    }
                }
                if(next_expandable_block_ptrs.empty()) break;
                std::swap(expandable_block_ptrs, next_expandable_block_ptrs);
            }
        }

    public:

        Pointis<N> surface_nodes_;

        std::vector<Pointis<N> > all_direction_local_moves_;

        std::vector<PathLen> dist_map_;

        std::vector<Pointi<N> > closet_pt_map_;

        Pointis<N> local_minimal_pts_;

        // some local minimal are very close, merge them to a isolated local minimal pt
        Pointis<N> isolated_minimal_pts_;

        AutoSortHeapPtr<PathLen, Id> self_sort_heap_;

        GridPtrs<N> corner_grids_;

        std::vector<PathLen> angle_offset_map_;

        std::vector<Id> obstacle_region_map_;

        // whether current grid is hyper node,
        std::vector<bool> hyper_map_;

        Pointis<N> error_pts_;

        // TODO: hyper id map, nearby hyper node have the same hyper id

    };

    template <Dimension N>
    using BlockDetectorInterfacePtr = std::shared_ptr<BlockDetectorInterface<N> >;

    class BlockDetectorOctoMap : public BlockDetectorInterface<3> {
    public:
        explicit BlockDetectorOctoMap(DimensionLength* dimension_info,
                             const IS_OCCUPIED_FUNC<3>& is_occupied,
                             const GridPtrs<3>& corner_grids,
                             const Pointis<3>& occ_pts,
                             PathLen minimum_block_width = 10,
                             const std::string& file_path = "",
                             bool force_update = false)
                             : BlockDetectorInterface<3>(dimension_info, is_occupied, file_path, minimum_block_width) {
            occ_pts_ = occ_pts;
            corner_grids_ = corner_grids;
            if(!force_update && this->deserialize(file_path)) {
                std::cout << "-- load " << this->all_block_ptrs_.size() << " blocks from " << file_path << " success " << std::endl;
            } else {
                detectBlock();
                if(this->serialize(file_path)) {
                    std::cout << "-- save blocks success " << std::endl;
                } else {
                    std::cout << "-- save blocks failed " << std::endl;
                }
            }
        }

        void detectBlock() override {
            std::cout << "-- BlockDetectorOctoMap load blocks from " << this->file_path_ << " failed, try to detect" << std::endl;
            octomap::OcTree ot(1.0);
            ot.setBBXMin(octomap::point3d(0,0,0));
            ot.setBBXMax(octomap::point3d(this->dimension_info_[0]-1,
                                          this->dimension_info_[1]-1,
                                          this->dimension_info_[2]-1));
            // only set occ grid and corner grid is not ok
//            for(const auto& occ_pt : occ_pts_) {
//                ot.updateNode(octomap::point3d(occ_pt[0], occ_pt[1], occ_pt[1]), true);
//            }
//            for(const auto& corner_pt : corner_grids_) {
//                ot.updateNode(octomap::point3d(corner_pt->pt_[0], corner_pt->pt_[1], corner_pt->pt_[1]), false);
//            }

            Pointi<3> pt;
            for(int x=0; x<dimension_info_[0]; x++) {
                for(int y=0; y<dimension_info_[1]; y++) {
                    for(int z=0; z<dimension_info_[2]; z++) {
                        pt[0] = x, pt[1] = y, pt[2] = z;
                        ot.updateNode(octomap::point3d(x, y, z), is_occupied_(pt));
                    }
                }
            }
            ot.updateInnerOccupancy();
            this->block_ptr_map_ = BlockPtrs<3>(getTotalIndexOfSpace<3>(this->dimension_info_), nullptr);
            // traversal all leaf nodes
            //std::cout << __FUNCTION__ << " ";
            for(auto iter = ot.begin_leafs(); iter != ot.end_leafs(); iter ++) {
                Pointi<3> pt;
                pt[0] = iter.getX(); pt[1] = iter.getY(); pt[2] = iter.getZ();
                if(iter.getDepth() == 16) continue;
                if(this->is_occupied_(pt)) continue;
//                std::cout << pt << " depth " << iter.getDepth() << " ";
//                std::cout << " / size " << iter.getSize() << " / occ " << iter->getOccupancy() << std::endl;


                // set block
                BlockPtr<3> block_ptr = std::make_shared<Block<3> >();
                //block_ptr->center_pt_ = center_pt;
                Id id = PointiToId(pt, this->dimension_info_);
                int half_width = pow(2, 16 - 1 - iter.getDepth());//floor(dist_map_[id]/sqrt(N)); // floor to ensure safety

                // set corner point of the cube
                Pointi<3> all_1_pt = GetFloorOrCeilFlag<3>().back();
                //std::cout << " pt " << pt << " depth " << iter.getDepth();
                block_ptr->min_ = pt - all_1_pt*half_width;
                block_ptr->max_ = pt + all_1_pt*(half_width-1);
                this->all_block_ptrs_.push_back(block_ptr);
                //std::cout << " min " << block_ptr->min_ << " max " << block_ptr->max_ << " occ " << iter->getOccupancy() << std::endl;
                if(isOutOfBoundary(block_ptr->min_, this->dimension_info_) || isOutOfBoundary(block_ptr->max_, this->dimension_info_)) {
                    std::cout << "FATAL: out of boundary in octomap block detector " << std::endl;
                }
                block_ptr->updateDimension();

                DimensionLength* dims = block_ptr->dimen_;

                Id total_count = getTotalIndexOfSpace<3>(dims);
                Pointi<3> new_pt;
                Id new_id;
                for(Id id=0; id<total_count; id++) {
                    new_pt = IdToPointi<3>(id, dims) + block_ptr->min_;
                    new_id = PointiToId(new_pt, this->dimension_info_);
                    //std::cout << "pt " << new_pt << " id " << new_id << std::endl;
                    if(this->block_ptr_map_[new_id] != nullptr) {
                        std::cout << " overlap block in " << __FUNCTION__ << std::endl;
                    }
                    this->block_ptr_map_[new_id] = block_ptr;
                }
            }
            std::cout << "-- BlockDetectorOctoMap detect " << this->all_block_ptrs_.size() << " blocks " << std::endl;
//            std::string file_path = "home/yaozhuo/octomap.bt";
//            std::ifstream vis_bin;
//            vis_bin.open(file_path);//, std::ios_base::in | std::ios_base::binary);
//            ot.writeBinary(file_path);
//            vis_bin.close();
        }

        Pointis<3> occ_pts_;

        GridPtrs<3> corner_grids_;

//        IS_OCCUPIED_FUNC<N> is_occupied_;
//
//        DimensionLength *dimension_info_;
//
//        BlockPtrs<N> block_ptr_map_; // store whether the grid in block; if not in block, ptr = nullptr
//
//        BlockPtrs<N> all_block_ptrs_;

    };

}
#endif //FREENAV_BLOCK_DETECT_H
