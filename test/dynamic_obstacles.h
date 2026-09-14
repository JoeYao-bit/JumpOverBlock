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
            //pre_map_     = std::vector<bool>(getTotalIndexOfSpace<N>(dim), false);
            current_map_ = std::vector<bool>(getTotalIndexOfSpace<N>(dim), false);

            previous_center_pts_.resize(obstacles.size(), Pointi<N>());
            current_center_pts_.resize(obstacles.size(), Pointi<N>());

            cur_occ_ids_ = {};
            pre_occ_ids_ = {};

            for(int i=0; i<obstacles_.size(); i++) {
                Pointi<N> center_pt = previous_center_pts_[i];
                for(int d=0; d<N; d++) {
                    center_pt[d] = rand() % dim_[d];
                }
                current_center_pts_.push_back(center_pt);
            }

            // construct local update isoc
            auto is_occupied_temp = [&](const Pointi<N> & pt) -> bool {
                if(isOutOfBoundary(pt, dim_)) {
                    return true;
                }
                return false;
            };
            isoc_ = is_occupied_temp;
        }

        // update each obstacle's center to random point in the space
        void random(int max_random_move_distance = 0) {
            std::cout << "-- max_random_move_distance = " << max_random_move_distance << std::endl;
            previous_center_pts_ = current_center_pts_;
            //pre_map_ = current_map_;
            pre_occ_ids_ = cur_occ_ids_;

            new_occ_pts_.clear();
            new_free_pts_.clear();

            Id total_index = getTotalIndexOfSpace<N>(dim_);

            current_map_ = std::vector<bool>(total_index, false);

            current_center_pts_.clear();
//            srand(time(0)); // use time as seed of generate random number
            // 获取当前时间（高精度）
            auto now = std::chrono::high_resolution_clock::now();
            // 将当前时间转换为微秒
            auto us = std::chrono::time_point_cast<std::chrono::microseconds>(now);
            auto value = us.time_since_epoch();
            long long micros = value.count();
            // 将微秒数转换为unsigned int（取模以适配，或者直接截断，因为unsigned int可能只有32位）
            unsigned int seed = static_cast<unsigned int>(micros);
            // 设置种子
            std::srand(seed);
            //std::cout << " seed = " << seed << std::endl;

            //std::cout << " time(0) = " << time(0) << std::endl;
            for(int i=0; i<obstacles_.size(); i++) {
                Pointi<N> center_pt = previous_center_pts_[i];
                //std::cout << "before / after sample: " << center_pt << " / ";
                for(int d=0; d<N; d++) {
                    if(max_random_move_distance == 0) {
                        center_pt[d] = rand() % dim_[d];
                    } else {
                        center_pt[d] = center_pt[d] + max_random_move_distance - rand() % (2*max_random_move_distance);
                        // avoid negative number
                        while(center_pt[d] < 0) {
                            center_pt[d] = center_pt[d] + dim_[d];
                        }
                        // avoid out of range
                        center_pt[d] = center_pt[d] % dim_[d];
                    }
                }
                std::cout << center_pt << std::endl;
                current_center_pts_.push_back(center_pt);
            }

            //std::cout << std::endl;

            occ_pt_count_ = 0;
            cur_occ_ids_.clear();
            for(int i=0; i<obstacles_.size(); i++) {
                Pointis<N> occupation = obstacles_[i]->getOccupiedGrid(current_center_pts_[i]);
                for(const auto& occupy_pt : occupation) {
                    if(!isOutOfBoundary(occupy_pt, dim_)) {
                        Id id = PointiToId(occupy_pt, dim_);
                        cur_occ_ids_.insert(id);
                        if(current_map_[id] == false) {
                            occ_pt_count_ ++;
                        }
                        current_map_[id] = true;
                    }
                }
            }

            for(const auto& cur_id : cur_occ_ids_) {
                if(pre_occ_ids_.find(cur_id) == pre_occ_ids_.end()) {
                    new_occ_pts_.push_back(IdToPointi<N>(cur_id, dim_));
                }
            }

            for(const auto& pre_id : pre_occ_ids_) {
                if(cur_occ_ids_.find(pre_id) == cur_occ_ids_.end()) {
                    new_free_pts_.push_back(IdToPointi<N>(pre_id, dim_));
                }
            }

            //std::cout << "-- new_occ/free_pts_ size = " << new_occ_pts_.size() << "/" << new_free_pts_.size() << std::endl;

            // construct local update isoc
            auto is_occupied_temp = [&](const Pointi<N> & pt) -> bool {
                if(isOutOfBoundary(pt, dim_)) {
                    return true;
                }
                return current_map_[PointiToId(pt, dim_)];
            };
            isoc_ = is_occupied_temp;


            //debug
//            std::vector<bool> copy_pre_map = pre_map_;
//            for(const auto& new_occ : new_occ_pts_) {
//                Id id = PointiToId(new_occ, dim_);
//                copy_pre_map[id] = true;
//            }
//            for(const auto& new_free : new_free_pts_) {
//                Id id = PointiToId(new_free, dim_);
//                copy_pre_map[id] = false;
//            }
//            for(Id id=0; id<total_index; id++) {
//                assert(copy_pre_map[id] == current_map_[id]);
//            }
//            std::cout << "Dynamic random pass debug" << std::endl;
        }

//        Pointis<N> getPreviousOccupationPoints() const {
//            return previous_occupied_pts_;
//        }
//
//        Pointis<N> getCurrentOccupationPoints() const {
//            return current_occupied_pts_;


        Pointis<N> getNewPassablePoints() const {
            return new_free_pts_;
        }

        Pointis<N> getNewOccupiedPoints() const {
            return new_occ_pts_;
        }

        int numOfObstacles() const {
            return obstacles_.size();
        }

        IS_OCCUPIED_FUNC<N> isoc_;

//        std::vector<bool> pre_map_;
        std::vector<bool> current_map_;

        Pointis<N> previous_center_pts_;
        Pointis<N> current_center_pts_;

        std::set<Id> pre_occ_ids_, cur_occ_ids_;

        Pointis<N> new_occ_pts_;
        Pointis<N> new_free_pts_;

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
