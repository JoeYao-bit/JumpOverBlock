//
// Created by yaozhuo on 2025/4/29.
//

#ifndef JUMPOVERBLOCK_DYNAMIC_OBSTACLES_H
#define JUMPOVERBLOCK_DYNAMIC_OBSTACLES_H

#include "freeNav-base/basic_elements/point.h"

namespace freeNav::JOB {

    template<Dimension N>
    struct Obstacle {

        virtual Pointis<N> getOccupiedGrid(const Pointi<N>& center_pt) const {
            Pointis<N> retv;
            for(const auto& pt : occ_pts_) {
                retv.push_back(center_pt + pt);
            }
            return retv;
        }

        Pointis<N> occ_pts_; // precomputation of occupied grid

    };

    template<Dimension N>
    using ObstaclePtr = std::shared_ptr<Obstacle<N> >;

    template<Dimension N>
    using ObstaclePtrs = std::vector<ObstaclePtr<N> >;

    template<Dimension N>
    struct CircleObstacle : public Obstacle<N> {

        explicit CircleObstacle(const float& radius) : radius_(radius) {
            DimensionLength local_dim[N];
            for(int d=0; d<N; d++) { local_dim[d] = 2*ceil(radius) + 1; }
            Id local_total_index = getTotalIndexOfSpace<N>(local_dim);
            Pointi<N> local_pt, local_center;
            local_center.setAll(ceil(radius));
            for(Id id=0; id<local_total_index; id++) {
                local_pt = IdToPointi<N>(id, local_dim) - local_center;
                if(local_pt.Norm() <= radius_) {
                    this->occ_pts_.push_back(local_pt);
                }
            }
        }

        float radius_;

    };

    template<Dimension N>
    using CircleObstaclePtr = std::shared_ptr<CircleObstacle<N> >;

    template<Dimension N>
    struct BlockObstacle : public Obstacle<N> {

        // min pt in the block is (0,0), max pt is in the diagonal corner
        explicit BlockObstacle(const Pointi<N>& max) : max_(max) {
            DimensionLength local_dim[N];
            for(int d=0; d<N; d++) { local_dim[d] = max_[d]; }
            Id local_total_index = getTotalIndexOfSpace<N>(local_dim);
            Pointi<N> local_pt, global_pt;
            for(Id id=0; id<local_total_index; id++) {
                local_pt = IdToPointi<N>(id, local_dim);
                this->occ_pts_.push_back(local_pt);
            }
        }

        Pointi<N> max_;

    };

    template<Dimension N>
    using BlockObstaclePtr = std::shared_ptr<BlockObstacle<N> >;

    // a DynamicObstacles contain some random circle obstacles and block obstacles
    template<Dimension N>
    struct DynamicObstacles {

        // dim limit the range move range of obstacle
        DynamicObstacles(DimensionLength* dim, const ObstaclePtrs<N>& obstacles)
        : dim_(dim), obstacles_(obstacles) {
            // init obstacle's position
        }

        // update each obstacle's center to random point in the space
        void random() {
            previous_center_pts_ = current_center_pts_;
            previous_occupied_pts_ = current_occupied_pts_;
            current_center_pts_.clear();
            srand(time(0)); // use time as seed of generate random number
            for(int i=0; i<obstacles_.size(); i++) {
                Pointi<N> center_pt;
                for(int d=0; d<N; d++) {
                    center_pt[d] = rand() % dim_[d];
                }
                current_center_pts_.push_back(center_pt);
            }
            current_occupied_pts_.clear();
            for(int i=0; i<obstacles_.size(); i++) {
                Pointis<N> occupation = obstacles_[i]->getOccupiedGrid(current_center_pts_[i]);
                for(const auto& occupy_pt : occupation) {
                    if(!isOutOfBoundary(occupy_pt, dim_)) {
                        current_occupied_pts_.push_back(occupy_pt);
                    }
                }
            }
        }

        Pointis<N> getPreviousOccupationPoints() const {
            return previous_occupied_pts_;
        }

        Pointis<N> getCurrentOccupationPoints() const {
            return current_occupied_pts_;
        }

        Pointis<N> previous_occupied_pts_;
        Pointis<N> current_occupied_pts_;

        Pointis<N> previous_center_pts_;
        Pointis<N> current_center_pts_;

        DimensionLength* dim_;
        ObstaclePtrs<N> obstacles_;

    };

}

#endif //JUMPOVERBLOCK_DYNAMIC_OBSTACLES_H
