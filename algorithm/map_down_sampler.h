//
// Created by yaozhuo on 2023/4/17.
//

#ifndef FREENAV_MAP_DOWN_SAMPLER_H
#define FREENAV_MAP_DOWN_SAMPLER_H

#include "rim_jump/basic_elements/point.h"

namespace freeNav::RimJump {

    template<Dimension N>
    class MapDownSampler {

    public:

        explicit MapDownSampler(const IS_OCCUPIED_FUNC<N> &is_occupied,
                                DimensionLength *dimension_info,
                                int down_sample_level) {
            down_sample_level_ = down_sample_level;
            raw_dimension_info_ = dimension_info;
            raw_is_occupied_ = is_occupied;
            shrink_size_ = pow(2, N);
            total_index_ = getTotalIndexOfSpace<N>(raw_dimension_info_);
            downSampleDimension();
            downSample();
        }

        ~MapDownSampler() {
            for(auto& dimension_info : dimension_infos_) {
                if(dimension_info != nullptr) {
                    delete dimension_info;
                }
            }
        }

        bool isOccupied(const Pointi<N> &pt, const int &down_sample_level) const {
            if (down_sample_level > down_sample_level_) {
                std::cerr << " down sample level " << down_sample_level
                          << " out of boundary (0, " << down_sample_level_ << ")" << std::endl;
            }
            if (down_sample_level == 0) {
                return raw_is_occupied_(pt);
            } else {
                const Id &id = downSamplePoint(pt, down_sample_level);
                //std::cout << pt << " / id " << id << " / " << down_sample_level << std::endl;
                return map_pyramid_[down_sample_level - 1][id];
            }
        }

        bool isOccupiedMiniMap(const Pointi<N> &pt) const {
            if(isOutOfBoundary(pt, dimension_infos_.back())) return true;
            Id new_id = PointiToId(pt, dimension_infos_.back());
            //std::cout << "pt " << pt << " / new id = " << new_id << std::endl;
            return map_pyramid_.back()[new_id];
        }

        inline Id downSamplePoint(const Pointi<N> &pt, const int &down_sample_level) const {
            Pointi<N> new_pt;
            for(int i=0; i<N; i++) {
                new_pt[i] = floor(pt[i]/pow(2, down_sample_level));
            }
            //std::cout << " raw pt " << pt << " / new_pt " << new_pt << std::endl;
            Id new_id = PointiToId(new_pt, dimension_infos_[down_sample_level-1]);
            return new_id;
        }

        void downSampleDimension() {
            for(int i=0; i<down_sample_level_; i++) {
                DimensionLength* dimen = new DimensionLength[N];
                for(int dim=0; dim<N; dim++) {
                    dimen[dim] = ceil(raw_dimension_info_[dim]/pow(2, i + 1));
                }
                dimension_infos_.push_back(dimen);
            }
        }

    //private:

        void downSample() {
            map_pyramid_.clear();
            Pointi<N> pt, pre_level_pt;
            Pointis<N> offsets = GetFloorOrCeilFlag<N>();
            for (int level = 0; level < down_sample_level_; level++) {
                int current_level_count = getTotalIndexOfSpace<N>(dimension_infos_[level]);
                //std::cout << " current_level_count = " << current_level_count << std::endl;
                std::vector<bool> level_map(current_level_count, false);
                for (int i = 0; i < current_level_count; i++) {
                    if (level == 0) {
                        pt = IdToPointi<N>(i, dimension_infos_[0]);
                        for(const auto& offset : offsets) {
                            Pointi<N> pre_level_pt = pt*2 + offset;
                            if(!isOutOfBoundary(pre_level_pt, raw_dimension_info_) && raw_is_occupied_(pre_level_pt)) {
                                level_map[i] = true;
                                break;
                            }
                        }
                    } else {
                        pt = IdToPointi<N>(i, dimension_infos_[level]);
                        for(const auto& offset : offsets) {
                            pre_level_pt = pt*2 + offset;
                            if(!isOutOfBoundary(pre_level_pt, dimension_infos_[level-1])) {
                                if(map_pyramid_[level - 1][PointiToId(pre_level_pt,  dimension_infos_[level-1])]) {
                                    level_map[i] = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                map_pyramid_.push_back(level_map);
            }
        }

        IS_OCCUPIED_FUNC<N> raw_is_occupied_;

        DimensionLength *raw_dimension_info_;

        std::vector<DimensionLength*> dimension_infos_;

        std::vector<std::vector<bool> > map_pyramid_;

        int down_sample_level_;

        Id shrink_size_;

        Id total_index_;
    };

    template<Dimension N>
    class MapDownSamplerSparse {

    public:

        explicit MapDownSamplerSparse(DimensionLength *dimension_info,
                                      const Pointis<N>& occ_grids,
                                      int shrink_level) {
            shrink_level_ = shrink_level;
            dimen_shrink_ = new DimensionLength[N];
            for(int dim=0; dim<N; dim++) {
                dimen_shrink_[dim] = dimension_info[dim]/shrink_level_;
            }
            Pointi<N> buf;
            for(const auto& pt : occ_grids) {
                for(int dim=0; dim<N; dim++) {
                    buf[dim] = pt[dim]/shrink_level_;
                }
                if(isOutOfBoundary(buf, dimen_shrink_)) { continue; }
                occ_grids_shrink_.push_back(buf);
                occ_ids_shrink_.insert(PointiToId(buf, dimen_shrink_));
            }
        }

        const Pointis<N>& getOccPtsShrink() {
            return occ_grids_shrink_;
        }

        const IdSet& getOccIdsShrink() {
            return occ_ids_shrink_;
        }

        ~ MapDownSamplerSparse () {
            if(dimen_shrink_) {
                delete[] dimen_shrink_;
            }
        }

        DimensionLength* getDimensionLengthShrink() {
            return dimen_shrink_;
        }

    private:



        int shrink_level_;

        Pointis<N> occ_grids_shrink_;

        IdSet occ_ids_shrink_;

        DimensionLength* dimen_shrink_;

    };

};

#endif //FREENAV_MAP_DOWN_SAMPLER_H
