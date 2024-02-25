//
// Created by yaozhuo on 2023/7/19.
//

#ifndef FREENAV_JOB_BLOCK_DETECTOR_GREEDY_H
#define FREENAV_JOB_BLOCK_DETECTOR_GREEDY_H

#include "block_detect.h"

namespace freeNav::JOB {


    template<Dimension N>
    class BlockDetectorGreedy : public BlockDetectorInterface<N> {
    public:
        explicit BlockDetectorGreedy(DimensionLength* dimension_info,
                               const IS_OCCUPIED_FUNC<N>& is_occupied,
                               const Pointis<N>& corner_grids,
                               PathLen minimum_block_width = 10,
                               const std::string& file_path = "",
                               bool force_update = false) :
                BlockDetectorInterface<N>(dimension_info, is_occupied, file_path, minimum_block_width, force_update) {
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

        Id getPtWithMaxDistToObstacle() {
            Id total_index = getTotalIndexOfSpace<N>(this->dimension_info_);
            PathLen max_dist_to_obstacle = 0;
            Id max_dist_id = MAX<Id>;
            for(Id id = 0; id < total_index; id++) {
                if(this->dist_map_updated_[id] == MAX<PathLen>) { continue; }
                if(max_dist_to_obstacle < this->dist_map_updated_[id]) {
                    max_dist_to_obstacle = this->dist_map_updated_[id];
                    max_dist_id = id;
                }
            }
            return max_dist_id;
        }

        virtual void detectCornerNodes(const GridPtrs<N>& corner_grids) {

        }

        virtual bool isAllExpansionStop(const std::vector<Pointis<N> > & all_direction_grids) {
            for(const auto & each_direction : all_direction_grids) {
                if(!each_direction.empty()) {
                    return false;
                }
            }
            return true;
        }

        // expand dist to nearest node from each tangent node in graph
        // considering limit range of expansion for acceleration ?
        virtual void waveFront() {
            dist_map_ = std::vector<PathLen>(getTotalIndexOfSpace<N>(this->dimension_info_), MAX<PathLen>);
            closet_pt_map_ = std::vector<Pointi<N> >(getTotalIndexOfSpace<N>(this->dimension_info_));
            this->block_ptr_map_ = BlockPtrs<N>(getTotalIndexOfSpace<N>(this->dimension_info_), nullptr);
            // set dist of tangent node to obstacle to 1
            for(const auto& pt : surface_nodes_)
            {
                //const auto& pt = tangent_nodes_.back();
                Id id = PointiToId(pt, this->dimension_info_);
                dist_map_[id] = 0;
                closet_pt_map_[id] = pt;
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

        void expandDistMapFromBlock(const BlockPtr<N>& block_ptr) {
            Pointis<N> surface_pts; // point on block surface
            DimensionLength* block_dim = block_ptr->dimen_;
            Pointi<N-1> plain_pt;
            Pointi<N> new_plain_pt;
            Id new_plain_pt_id;
            //std::cout << __FUNCTION__ << " block_boundary " << block_ptr->min_ << block_ptr->max_ << std::endl;
            for(int dim = 0; dim < 2*N; dim ++) {
                DimensionLength plain[N-1];
                for(Dimension i=0; i<dim/2; i++) {
                    plain[i] = block_dim[i];
                }
                for(Dimension i=dim/2+1; i<N; i++) {
                    plain[i-1] = block_dim[i];
                }
                // get the dimension of the N-1 dimension plain
                Id total_plain_index = getTotalIndexOfSpace<N-1>(plain);
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
                        continue;
                    }
                    dist_map_updated_[new_plain_pt_id] = 0;
                    closet_pt_map_updated_[new_plain_pt_id] = new_plain_pt;
                    surface_pts.push_back(new_plain_pt);
                }
            }
//            std::cout << " surface_pts size " << surface_pts.size() << std::endl;
//            std::cout << " surface_pts " << surface_pts << std::endl;
            //return;
            std::vector<Pointis<N> > all_direction_grids(2 * N, surface_pts);//{tangent_node});
            Pointis<N> next_expansion;
            while (!isAllExpansionStop(all_direction_grids)) {
                // each tangent node expand simultaneously
                for (Dimension dim = 0; dim < N * 2; dim++) {
                    next_expansion.clear();
                    for (const auto &current_pt : all_direction_grids[dim]) {
                        const auto& current_pt_expansion = expandCurrentPoint(current_pt, dim, true);
                        next_expansion.insert(next_expansion.end(), current_pt_expansion.begin(),
                                              current_pt_expansion.end());
                    }
                    std::swap(all_direction_grids[dim], next_expansion);
                }
            }
        }

        // expand from single local minimal
        virtual BlockPtr<N> createInitBlock(const Pointi<N>& center_pt, int half_width = 0) {
            BlockPtr<N> block_ptr = std::make_shared<Block<N> >();
            //block_ptr->center_pt_ = center_pt;
            Id id = PointiToId(center_pt, this->dimension_info_);
            if(half_width == 0) {
                half_width = std::max(1, (int)floor(this->dist_map_updated_[id] / sqrt(N)) - 1);//floor(dist_map_[id]/sqrt(N)); // floor to ensure safety
            }
            // set corner point of the cube
            Pointi<N> all_1_pt = GetFloorOrCeilFlag<N>().back();

            block_ptr->min_ = center_pt - all_1_pt*half_width;
            block_ptr->max_ = center_pt + all_1_pt*half_width;
            block_ptr->updateDimension();
//            std::cout << " center_pt " << center_pt << std::endl;
//            std::cout << " dist_to_obstacle " << this->dist_map_updated_[id] << std::endl;
//            std::cout << " block_ptr->min_/max_ " << block_ptr->min_ << " / " << block_ptr->max_ << std::endl;

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
                this->dist_map_updated_[new_id] = MAX<PathLen>;
            }
            return block_ptr;
        }

        virtual Pointis<N> expandCurrentPoint(const Pointi<N>& current_pt, int dim, bool updated = false) {
            // expand current point
            Pointis<N> next_expansion;
            Id current_id = PointiToId(current_pt, this->dimension_info_);
            Pointi<N> new_pt;
            Id new_id;
            PathLen new_dist;
            auto& dist_map = updated ? dist_map_updated_ : dist_map_;
            auto& closet_pt_map = updated ? closet_pt_map_updated_ : closet_pt_map_;
            for(const auto& offset : all_direction_local_moves_[dim])
            {
                new_pt = current_pt + offset;
                if(this->is_occupied_(new_pt)) continue;
                new_id = PointiToId(new_pt, this->dimension_info_);
                if(this->block_ptr_map_[new_id] != nullptr) continue;
                // stop expansion when hit obstacle, smaller dist to obstacle
                if(dist_map[new_id] == MAX<PathLen>) {
                    //std::cout << " closet_pt_map_[current_id] " << closet_pt_map_[current_id] << std::endl;
                    new_dist = (new_pt-closet_pt_map[current_id]).Norm();
                    dist_map[new_id] = new_dist;
                    closet_pt_map[new_id] = closet_pt_map[current_id];
                    next_expansion.push_back(new_pt);
                } else {
                    //std::cout << " closet_pt_map_[new_id] " << closet_pt_map_[new_id] << std::endl;
                    new_dist = (new_pt-closet_pt_map[current_id]).Norm();
                    // considering the order of expansion, allow equal to considering expansion of two nearby corner
                    // with out equal, may cause slit that never visit
                    if(new_dist < dist_map[new_id]) {
                        dist_map[new_id] = new_dist;
                        closet_pt_map[new_id] = closet_pt_map[current_id];
                        next_expansion.push_back(new_pt);
                    }
                }
                // when occurred small dist, mark a local minimal
            }
            return next_expansion;
        }



        void detectBlock() override {
            std::cout << "-- BlockDetector load blocks from " << this->file_path_ << " failed, try to detect" << std::endl;
            this->all_direction_local_moves_ = initAllDirectionLocalMoves<N>();
            surface_nodes_ = corner_grids_;
            //std::cout << " this->corner_grids_ size " << corner_grids_.size() << " / " << surface_nodes_.size() << std::endl;
            waveFront();
            dist_map_updated_.clear();
            std::swap(dist_map_updated_, dist_map_);
            closet_pt_map_updated_.clear();
            std::swap(closet_pt_map_updated_, closet_pt_map_);
            int minimum_radius = ceil(this->min_block_width_*sqrt(N));
            int count = 0;
            while(1) {
                // determine point with max dist to obstacle
                Id max_dist_id = getPtWithMaxDistToObstacle();
                //std::cout << " max_dist_id " << max_dist_id << " / this->dist_map_updated_[max_dist_id] " << this->dist_map_updated_[max_dist_id] << std::endl;
                // if no more room to generate block, stop
                if(this->dist_map_updated_[max_dist_id] < minimum_radius) { break; }
                // expand from the point till no more room
                Pointi<N> center_pt = IdToPointi<N>(max_dist_id, this->dimension_info_);
                BlockPtr<N> block_ptr = createInitBlock(center_pt);
                this->all_block_ptrs_.push_back(block_ptr);
                BlockPtrs<N> expandable_block_ptrs = {block_ptr};
                while(1) {
                    BlockPtrs<N> next_expandable_block_ptrs;
                    for(const auto& block_ptr : expandable_block_ptrs) {
                        if(expandBlockOneStep(block_ptr)) {
                            next_expandable_block_ptrs.push_back(block_ptr);
                        }
                    }
                    if(next_expandable_block_ptrs.empty()) { break; }
                    std::swap(expandable_block_ptrs, next_expandable_block_ptrs);
                }
                //break;
                // update dist map, from current block
                expandDistMapFromBlock(block_ptr);
                //if(count > 0) break;
                count ++;
            }

            std::cout << "-- BlockDetector detect " << this->all_block_ptrs_.size() << " blocks " << std::endl;

        }

        virtual bool expandBlockOneStep(const BlockPtr<N>& block_ptr) {
            bool is_expanded = false;
            for(Dimension dim=0; dim<2*N; dim++) {
                if(block_ptr->is_expandable_[dim] && isLegalToExpandBlockInDirectionOneDirection(block_ptr, dim)) {
                    is_expanded = true;
                }
            }
            return is_expanded;
        }

        // if is legal, update with new plain block ptr in the ptr map
        virtual bool isLegalToExpandBlockInDirectionOneDirection(const BlockPtr<N>& block_ptr, int dim) {
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
                this->dist_map_updated_[new_pt_id] = MAX<PathLen>;
            }
            if(dim%2 == 1) {
                block_ptr->max_[dim/2] ++;
            } else {
                block_ptr->min_[dim/2] --;
            }
            block_ptr->updateDimension();
            return true;
        }

        DimensionLength* dimension_length_shrink_;

        Pointis<N> surface_nodes_;

        std::vector<Pointis<N> > all_direction_local_moves_; // constant value, during iteration

        std::vector<PathLen> dist_map_;

        std::vector<PathLen> dist_map_updated_;

        std::vector<Pointi<N> > closet_pt_map_;

        std::vector<Pointi<N> > closet_pt_map_updated_;

        Pointis<N> local_maximal_pts_;

        // some local minimal are very close, merge them to a isolated local minimal pt
        Pointis<N> isolated_minimal_pts_;

        Pointis<N> corner_grids_;


    };

    template <Dimension N>
    using BlockDetectorGreedyPtr = std::shared_ptr<BlockDetectorGreedy<N> >;

    // detect block from shrink grid space
    template<Dimension N>
    class BlockDetectorGreedyWithShrink : public BlockDetectorInterface<N>  {
    public:
        explicit BlockDetectorGreedyWithShrink(DimensionLength* dimension_info,
                                               const IS_OCCUPIED_FUNC<N>& is_occupied,
                                               const Pointis<N>& occ_grids,
                                               int shrink_level,
                                               PathLen minimum_block_width = 10,
                                               const std::string& file_path = "",
                                               bool force_update = false)
             : BlockDetectorInterface<N>(dimension_info, is_occupied, file_path, minimum_block_width, force_update) {
            shrink_level_ = shrink_level;
            occ_grids_ = occ_grids;
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

        BlockDetectorGreedyPtr<N> initShrinkBlockDetector() {
            /*
             *         explicit BlockDetectorGreedy(DimensionLength* dimension_info,
                               const IS_OCCUPIED_FUNC<N>& is_occupied,
                               const GridPtrs<N>& corner_grids,
                               PathLen minimum_block_width = 10,
                               const std::string& file_path = "",
                               bool force_update = false)
             * */
            std::cout << "-- raw " << __FUNCTION__ << " shrink_level_ = " << shrink_level_ << ", with minimum_block_width = " << this->min_block_width_ << std::endl;
            MapDownSamplerSparse<N> shrink_space(this->dimension_info_, occ_grids_, shrink_level_);
            PathLen min_block_length_of_shrink_space = this->min_block_width_/shrink_level_;//std::max(this->min_block_width_/shrink_level_, (PathLen)2.);
            std::cout << "-- min block of shrink space = " << min_block_length_of_shrink_space << std::endl;
            auto dimension_shrink = shrink_space.getDimensionLengthShrink();
            Id total_index_shrink = getTotalIndexOfSpace<N>(dimension_shrink);
            std::vector<bool> grid_map_shrink(total_index_shrink, false);
            for(const auto& pt : shrink_space.getOccPtsShrink()) {
                if(isOutOfBoundary(pt, dimension_shrink)) {
                    std::cout << " error, out of boundary " << pt << std::endl;
                    continue;
                }
                Id id = PointiToId(pt, dimension_shrink);
                grid_map_shrink[id] = true;
            }
            IS_OCCUPIED_FUNC<N> is_occupied_func;
            auto is_occupied = [&](const Pointi<N> &pt) -> bool {
                if(isOutOfBoundary(pt, dimension_shrink)) { return true; }
                Id id = PointiToId(pt, shrink_space.getDimensionLengthShrink());
                return grid_map_shrink[id];
            };

            is_occupied_func = is_occupied;
            SET_OCCUPIED_FUNC<N> set_occupied_func;
            SurfaceProcessor<N> surface_shrink(dimension_shrink, is_occupied_func, set_occupied_func);
            surface_shrink.surfaceGridsDetection(true);
            BlockDetectorGreedyPtr<N> block_detector_ptr_shrink = std::make_shared<BlockDetectorGreedy<N> >(dimension_shrink,
                                                                is_occupied_func,
                                                                surface_shrink.getSurfacePts(),
                                                                min_block_length_of_shrink_space,
                                                                "",
                                                                false);
            return block_detector_ptr_shrink;
        }

        void detectBlock() override {
            block_detector_ptr_shrink_ = initShrinkBlockDetector();

            this->block_ptr_map_ = BlockPtrs<N>(getTotalIndexOfSpace<N>(this->dimension_info_), nullptr);
            for(const auto& shrink_block : block_detector_ptr_shrink_->all_block_ptrs_) {
                BlockPtr<N> block_ptr = inheritFromShrinkBlock(shrink_block);
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

        virtual BlockPtr<N> inheritFromShrinkBlock(const BlockPtr<N>& shrink_block) {
            BlockPtr<N> block_ptr = std::make_shared<Block<N> >();

            block_ptr->min_ = shrink_block->min_ * shrink_level_;
            Pointi<N> up_bound = shrink_block->max_ * shrink_level_;
            // boundary limitation
            for(int dim=0; dim<N; dim++) {
                block_ptr->max_[dim] = std::min((DimensionLength)up_bound[dim], this->dimension_info_[dim]-1);
            }
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
        virtual bool isLegalToExpandBlockInDirectionOneDirection(const BlockPtr<N>& block_ptr, int dim) {
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

        virtual bool expandBlockOneStep(const BlockPtr<N>& block_ptr) {
            bool is_expanded = false;
            for(Dimension dim=0; dim<2*N; dim++) {
                if(block_ptr->is_expandable_[dim] && isLegalToExpandBlockInDirectionOneDirection(block_ptr, dim)) {
                    is_expanded = true;
                }
            }
            return is_expanded;
        }

    private:

        BlockDetectorGreedyPtr<N> block_detector_ptr_shrink_;

        int shrink_level_ = 0;

        Pointis<N> occ_grids_;

    };


    template <Dimension N>
    using BlockDetectorGreedyWithShrinkPtr = std::shared_ptr<BlockDetectorGreedyWithShrink<N> >;

}

#endif //FREENAV_BLOCK_DETECTOR_GREEDY_H
