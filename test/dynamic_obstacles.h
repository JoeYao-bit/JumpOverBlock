//
// Created by yaozhuo on 2025/4/29.
//

#ifndef JUMPOVERBLOCK_DYNAMIC_OBSTACLES_H
#define JUMPOVERBLOCK_DYNAMIC_OBSTACLES_H

#include <random>
#include "freeNav-base/basic_elements/point.h"

namespace freeNav::JOB {

    float generateRandomFloat(float min, float max)
    {
        // 使用默认的随机设备创建种子
        std::random_device rd;

        // 使用种子初始化梅森旋转引擎
        std::mt19937 mt(rd());

        // 创建一个均匀分布，范围为[min, max]
        std::uniform_real_distribution<float> dist(min, max);

        // 生成一个位于范围内的随机浮点数
        return dist(mt);
    }


    template<Dimension N>
    struct Obstacle;

    template<Dimension N>
    using ObstaclePtr = std::shared_ptr<Obstacle<N> >;

    template<Dimension N>
    struct Obstacle {


        virtual Pointis<N> getOccupiedGrid(const Pointi<N>& center_pt) const {
            Pointis<N> retv;
            for(const auto& pt : occ_pts_) {
                retv.push_back(center_pt + pt);
            }
            return retv;
        }

        virtual std::string toStr() const = 0;

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

        std::string toStr() const override {
            std::stringstream ss;
            ss << "CircleOBS:" << radius_;
            return ss.str();
        }


        float radius_;

    };

    template<Dimension N>
    using CircleObstaclePtr = std::shared_ptr<CircleObstacle<N> >;

    template<Dimension N>
    using CircleObstaclePtrs = std::vector<CircleObstaclePtr<N> >;

    template<Dimension N>
    CircleObstaclePtr<N> generateRandomCircle(float min_radius, float max_radius) {
        return std::make_shared<CircleObstacle<N> >(generateRandomFloat(min_radius, max_radius));
    }


    template<Dimension N>
    CircleObstaclePtrs<N> generateRandomCircleObstacles(int count, float min_radius, float max_radius) {
        assert(count >= 0);
        CircleObstaclePtrs<N> retv;
        for(int i=0; i<count; i++) {
            retv.push_back(std::make_shared<CircleObstacle<N> >(generateRandomFloat(min_radius, max_radius)));
        }
        return retv;
    }



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

        std::string toStr() const override {
            std::stringstream ss;
            ss << "BlockObstacle: " << max_;
            return ss.str();
        }

        Pointi<N> max_;

    };

    template<Dimension N>
    using BlockObstaclePtr = std::shared_ptr<BlockObstacle<N> >;

    template<Dimension N>
    using BlockObstaclePtrs = std::vector<BlockObstaclePtr<N> >;

    template<Dimension N>
    BlockObstaclePtrs<N> generateRandomBlockObstacles(int count, const Pointi<N>& min_pt, const Pointi<N>& max_pt) {
        assert(count >= 0);
        BlockObstaclePtrs<N> retv;
        Pointi<N> temp_pt;
        for(int i=0; i<count; i++) {
            for(int d=0; d<N; d++) {
                temp_pt[d] = min_pt[d] + rand()%(max_pt[d] - min_pt[d]);
            }
            retv.push_back(std::make_shared<BlockObstacle<N> >(temp_pt));
        }
        return retv;
    }

    // a DynamicObstacles contain some random circle obstacles and block obstacles
    template<Dimension N>
    struct DynamicObstacles {

        // dim limit the range move range of obstacle
        DynamicObstacles(DimensionLength* dim, const ObstaclePtrs<N>& obstacles)
        : dim_(dim), obstacles_(obstacles) {
            // init obstacle's position
            map_ = std::vector<bool>(getTotalIndexOfSpace<N>(dim), false);
        }

        // update each obstacle's center to random point in the space
        void random() {
            previous_center_pts_ = current_center_pts_;
            previous_occupied_pts_ = current_occupied_pts_;

            for(const auto& pt : previous_occupied_pts_) {
                map_[PointiToId(pt, dim_)] = false;
            }

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
            occ_pt_count_ = 0;
            for(int i=0; i<obstacles_.size(); i++) {
                Pointis<N> occupation = obstacles_[i]->getOccupiedGrid(current_center_pts_[i]);
                for(const auto& occupy_pt : occupation) {
                    if(!isOutOfBoundary(occupy_pt, dim_)) {
                        current_occupied_pts_.push_back(occupy_pt);
                        Id id = PointiToId(occupy_pt, dim_);
                        if(map_[id] == false) {
                            occ_pt_count_ ++;
                        }
                        map_[id] = true;
                    }
                }
            }

            // construct local update isoc
            auto is_occupied_temp = [&](const Pointi<N> & pt) -> bool {
                if(isOutOfBoundary(pt, dim_)) {
                    return true;
                }
                return map_[PointiToId(pt, dim_)];
            };
            isoc_ = is_occupied_temp;
        }

        Pointis<N> getPreviousOccupationPoints() const {
            return previous_occupied_pts_;
        }

        Pointis<N> getCurrentOccupationPoints() const {
            return current_occupied_pts_;
        }

        int numOfObstacles() const {
            return obstacles_.size();
        }

        IS_OCCUPIED_FUNC<N> isoc_;

        std::vector<bool> map_;

        Pointis<N> previous_occupied_pts_;
        Pointis<N> current_occupied_pts_;

        Pointis<N> previous_center_pts_;
        Pointis<N> current_center_pts_;

        DimensionLength* dim_;
        ObstaclePtrs<N> obstacles_;

        int occ_pt_count_ = 0;

        template<Dimension T>
        friend  std::ostream& operator<< (std::ostream& os, const DynamicObstacles<T>& obs);
    };

    template<Dimension N>
    std::ostream& operator<<(std::ostream& os, const DynamicObstacles<N>& obs) {
        os << obs.obstacles_.size() << " dynamic Obstacles: ";
        for(int i=0; i<obs.obstacles_.size(); i++) {
            os << obs.obstacles_[i]->toStr() << " at " << obs.current_center_pts_[i] << "/ ";
        }
        return os;
    }



}

#endif //JUMPOVERBLOCK_DYNAMIC_OBSTACLES_H
