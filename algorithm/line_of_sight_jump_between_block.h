//
// Created by yaozhuo on 2023/4/29.
//

#ifndef FREENAV_JOB_LOS_JUMP_BETWEEN_BLOCK_H
#define FREENAV_JOB_LOS_JUMP_BETWEEN_BLOCK_H

#include "block_detect.h"
namespace freeNav::JOB {



    // return: current in the block or not
    // for a line that cross a block, find the point on it and leave obstacle
    // update inner index of line
    template<Dimension N>
    int findExitPointOfBlock(Line<int, N>& line, const Pointi<N>& current_pt, const int& index, const Pointi<N>& min_pt, const Pointi<N>& max_pt) {
        // check whether the line reach end of line
        if(index >= line.step - 1) { return 0; }
        //Pointi<N> current_pt = line.GetPoint(index);
        // check whether current line's last traveled point in the block
        //if(!block_ptr->PointiInBlock(current_pt)) {
        //std::cout << " not in block" << std::endl;
        //    return 0;
        //}
        //bool line_increase = (line.step_length > 0);
        //Dimension minimum_step_exit_dim = 0;
        Fraction minimum_step_to_exit = Fraction(line.step), future_step;
        // determine the fast dim to leave current block
        for(Dimension dim=0; dim<N; dim++) {
            if(line.parameter[dim].second == 0) {
                continue;
            } else {
                if(line.parameter[dim].second > 0) {
                    future_step = (Fraction(max_pt[dim] - current_pt[dim]) /
                                   line.parameter[dim].second).toAbs();
                } else {
                    future_step = (Fraction(min_pt[dim] - current_pt[dim]) /
                                   line.parameter[dim].second).toAbs();
                }
                //std::cout << " dim " << dim << " / future_step " << future_step.toFloat() << std::endl;
                if (future_step < minimum_step_to_exit) {
                    //minimum_step_exit_dim = dim;
                    minimum_step_to_exit = future_step;
                }
            }
        }
        //std::cout << " line.step " << line.step << " - index " << index << std::endl;
        //std::cout << " minimum_step_to_exit " << minimum_step_to_exit << std::endl;
        if(minimum_step_to_exit > line.step - index) {
            //std::cout << " reach end of line" << std::endl;
            return line.step - index;// - 1;
        }
        return std::max((line.step*minimum_step_to_exit - 1).floor(), 0);
    }

    // return: current in the block or not
    // for a line that cross a block, find the point on it and leave obstacle
    // update inner index of line
    template<typename T, Dimension N>
    int findExitPointOfBlock(Line<T, N>& line, const Pointi<N>& current_pt, const int& index, const BlockPtr<N>& block_ptr) {
        return findExitPointOfBlock(line, current_pt, index, block_ptr->min_, block_ptr->max_);
    }

    template <Dimension N>
    bool LineCrossObstacleWithBlockJump(const Pointi<N>& pt1, const Pointi<N>& pt2,
                                        BlockDetectorInterfacePtr<N> block_detector_ptr,
                                        std::vector<Pointi<N> >& visited_pt,
                                        int& count_of_block) {
        if(pt1 == pt2) { return true; }
        visited_pt.clear();
        count_of_block = 0;
        Line<int, N> line(pt1, pt2);
        int check_step = line.step;
        Pointi<N> pt;
        Id current_id;
        int jump_step = 0;
        for(int i=1; i<check_step; i++) {
            pt = line.GetPoint(i);
            visited_pt.push_back(pt);
            if(block_detector_ptr->is_occupied_(pt)) {
                return true;
            }
            //if(is_occupied(pt)) { return true; }
            current_id = PointiToId(pt, block_detector_ptr->dimension_info_);
            const auto& block_ptr = block_detector_ptr->block_ptr_map_[current_id];
            // if in block, jump over current block
            if(block_ptr != nullptr) {
                jump_step = findExitPointOfBlock(line, pt, i, block_ptr);
                //std::cout << " jump step " << jump_step << std::endl;
                i = i + jump_step;
                count_of_block ++;
            }
        }
        return false;
    }

    // 2026-09-08:发现引入跳过块后有tMaxX和tMaxY不更新的情况
    //  dimension_info = 65 81
//-- BlockDetector load blocks from  failed, try to detect
//-- BlockDetector detect 19 blocks
//-- save blocks failed
//-- block detect end in 0.972ms
//get point 21.6667, 19.7778
//get point 23.8889, 36.5556
//tMaxX/Y = 0.15/0.013245
//tMaxX/Y = 0.6/0.430463
//jump block line not collide
//get point 28.6667, 37.8889
//get point 27.7778, 56.7778
//tMaxX/Y = 0.750001/0.00588233
//tMaxX/Y = 0.750001/0.323529
//tMaxX/Y = 0.750001/0.376471
//tMaxX/Y = 0.750001/0.429412
//tMaxX/Y = 0.750001/0.482353
//tMaxX/Y = 0.750001/0.535294
//tMaxX/Y = 0.750001/0.588235
//tMaxX/Y = 0.750001/0.641177
//tMaxX/Y = 0.750001/0.694118
//tMaxX/Y = 0.750001/0.747059
//jump block line not collide
//get point 20.8889, 29.6667
//get point 11.1111, 19.7778
//tMaxX/Y = 0.0909091/0.0674157
//tMaxX/Y = 0.193182/0.269663
//tMaxX/Y = 0.193182/0.269663
//test_los_for_sparse_map: /home/yaozhuo/code/JumpOverBlock/test/../algorithm/line_of_sight_jump_between_block.h:217: bool freeNav::JOB::lineOfSightCheckAW(const freeNav::Point<T, 2>&, const freeNav::Point<T, 2>&, int, int, freeNav::IS_OCCUPIED_FUNC<2>&, BlockDetectorInterfacePtr<2>, std::vector<freeNav::Point<int, 2>, std::allocator<freeNav::Point<int, 2> > >&, std::vector<freeNav::Point<float, 2> >&) [with T = float; freeNav::IS_OCCUPIED_FUNC<2> = std::function<bool(const freeNav::Point<int, 2>&)>; BlockDetectorInterfacePtr<2> = std::shared_ptr<BlockDetectorInterface<2> >]: Assertion `0' failed.
//Signal: SIGABRT (Aborted)

    template<typename T>
    bool lineOfSightCheckAW(const Point<T,2>& start, const Point<T, 2>& end,
                            int gridW, int gridH, const IS_OCCUPIED_FUNC<2>& isoc,
                            JOB::BlockDetectorInterfacePtr<2> block_detector_ptr,
                            std::vector<Pointi<2> >& visited_pt,
                            std::vector<Pointf<2> >& visited_fpt)
    {
        visited_pt.clear();
        visited_fpt.clear();
        const float eps = 1e-6f;
        Point<T,2> dir{end[0] - start[0], end[1] - start[1]};

        Pointi<2> current_pt;
        Pointf<2> current_fpt;
        Id current_id;

        // 1. 起点终点完全重合
        if (std::fabs(dir[0]) < eps && std::fabs(dir[1]) < eps)
        {
            //std::cout << "case 0 " << std::endl;
            int x0 = static_cast<int>(std::floor(start[0]));
            int y0 = static_cast<int>(std::floor(start[1]));
            if (x0 >= 0 && x0 < gridW && y0 >= 0 && y0 < gridH) {
                visited_pt.push_back(Pointi<2>{x0, y0});
                if(isoc(Pointi<2>{x0, y0})) {
                    return true;
                }
            }
            return false;
        }

        // 2. 纯垂直线：x全程不变，只遍历y轴
        if (std::fabs(dir[0]) < eps)
        {
            //std::cout << "case 1 " << std::endl;
            int fixedX = static_cast<int>(std::floor(start[0]));
            float yMinF = std::min(start[1], end[1]);
            float yMaxF = std::max(start[1], end[1]);
            int yStart = static_cast<int>(std::floor(yMinF));
            int yEnd = static_cast<int>(std::floor(yMaxF));

            // for循环遍历所有y栅格
            for (int y = yStart; y <= yEnd; ++y)
            {
                // 栅格边界校验
                if (fixedX >= 0 && fixedX < gridW && y >= 0 && y < gridH)
                {
                    current_pt = Pointi<2>{fixedX, y};
                    visited_pt.push_back(current_pt);
                    if(isoc(Pointi<2>{fixedX, y})) {
                        return true;
                    }
                    current_id = PointiToId(current_pt, block_detector_ptr->dimension_info_);
                    BlockPtr<2> current_block = block_detector_ptr->block_ptr_map_[current_id];
                    if(current_block != nullptr) {
                        if (current_block->max_[1]+1 > yEnd) {
                            return false;
                        } else {
                            y = current_block->max_[1];
                        }
                    }
                } else {
                    return false;
                }
            }
            return false;
        }

        // 3. 纯水平线：y全程不变，只遍历x轴
        if (std::fabs(dir[1]) < eps)
        {
            //std::cout << "case 2 " << std::endl;
            int fixedY = static_cast<int>(std::floor(start[1]));
            float xMinF = std::min(start[0], end[0]);
            float xMaxF = std::max(start[0], end[0]);
            int xStart = static_cast<int>(std::floor(xMinF));
            int xEnd = static_cast<int>(std::floor(xMaxF));

            // for循环遍历所有x栅格
            for (int x = xStart; x <= xEnd; ++x)
            {
                // 栅格边界校验
                if (x >= 0 && x < gridW && fixedY >= 0 && fixedY < gridH)
                {
                    current_pt = Pointi<2>{x, fixedY};
                    visited_pt.push_back(current_pt);
                    if(isoc(Pointi<2>{x, fixedY})) {
                        return true;
                    }
                    current_id = PointiToId(current_pt, block_detector_ptr->dimension_info_);
                    BlockPtr<2> current_block = block_detector_ptr->block_ptr_map_[current_id];
                    if(current_block != nullptr) {
                        if (current_block->max_[0]+1 > xEnd) {
                            return false;
                        } else {
                            x = current_block->max_[0];
                        }
                    }
                }
            }
            return false;
        }
        //std::cout << "case 3 " << std::endl;

        // 当为斜线时
        // 当前栅格
        int x = static_cast<int>(std::floor(start[0]));
        int y = static_cast<int>(std::floor(start[1]));

        // 步进方向 ±1
        int stepX = dir[0] > 0.f ? 1 : -1;
        int stepY = dir[1] > 0.f ? 1 : -1;

        // tDelta：沿轴移动1格对应的t增量（t是沿射线的参数，0=起点，1=终点）
        float tDeltaX = std::fabs(1.f / dir[0]);
        float tDeltaY = std::fabs(1.f / dir[1]);

        // tMax：首次穿过网格线的t值
        float tMaxX, tMaxY;
        if (dir[0] > 0.f)
            tMaxX = (x + 1.f - start[0]) * tDeltaX;
        else
            tMaxX = (start[0] - x) * tDeltaX;

        if (dir[1] > 0.f)
            tMaxY = (y + 1.f - start[1]) * tDeltaY;
        else
            tMaxY = (start[1] - y) * tDeltaY;

        // 核心：t ∈ [0, 1] 代表线段范围，t=1精准对应终点

        Line<T, 2> line(start, end);
        float pre_tMaxX = 1e20, pre_tMaxY = 1e20;
        bool step_by_step = false;
        while (x >= 0 && x < gridW && y >= 0 && y < gridH)
        {
            //std::cout << "tMaxX/Y = " << tMaxX << "/" << tMaxY << std::endl;
            step_by_step = false;
            if(pre_tMaxX == tMaxX && pre_tMaxY == tMaxY) {
                step_by_step = true;
            }
            pre_tMaxX = tMaxX; pre_tMaxY = tMaxY;
            current_pt = Pointi<2>{x, y};
            visited_pt.push_back(current_pt);
            if(tMaxX < tMaxY) {
                current_fpt = Pointf<2>{start[0] + tMaxX*dir[0], start[1] + tMaxX*dir[1]};
            } else {
                current_fpt = Pointf<2>{start[0] + tMaxY*dir[0], start[1] + tMaxY*dir[1]};
            }
            visited_fpt.push_back(current_fpt);

            if(isoc(current_pt)) {
                return true;
            }
            current_id = PointiToId(current_pt, block_detector_ptr->dimension_info_);
            BlockPtr<2> current_block = block_detector_ptr->block_ptr_map_[current_id];
            if (!step_by_step && current_block != nullptr) {
                Pointi<2> block_min = current_block->min_;
                Pointi<2> block_max = current_block->max_+Pointi<2>{1,1}; // the real boundary of current block

                //std::cout << "block_max/min = " << block_max << "/" << block_min << std::endl;

                float t_candidate_x = 1e20f;
                float t_candidate_y = 1e20f;

                if(std::fabs(dir[0]) > eps)
                {
                    float b_min_x = static_cast<float>(block_min[0]);
                    float b_max_x = static_cast<float>(block_max[0]) + 1.0f;
                    float t_x_low  = (b_min_x - start[0]) / dir[0];
                    float t_x_high = (b_max_x - start[0]) / dir[0];
                    if(dir[0] > 0)
                        t_candidate_x = t_x_high;
                    else
                        t_candidate_x = t_x_low;
                }

                if(std::fabs(dir[1]) > eps)
                {
                    float b_min_y = static_cast<float>(block_min[1]);
                    float b_max_y = static_cast<float>(block_max[1]) + 1.0f;
                    float t_y_low  = (b_min_y - start[1]) / dir[1];
                    float t_y_high = (b_max_y - start[1]) / dir[1];
                    if(dir[1] > 0)
                        t_candidate_y = t_y_high;
                    else
                        t_candidate_y = t_y_low;
                }

                float t_block_exit = std::min(t_candidate_x, t_candidate_y);

                //std::cout << "t_block_exit = " << t_block_exit << std::endl;

                if(t_block_exit >= 1.0f - eps)
                {
                    return false;
                }

                Pointf<2> exit_pt{
                        start[0] + t_block_exit * dir[0],
                        start[1] + t_block_exit * dir[1]
                };

                // 更新跳跃后栅格索引
                x = static_cast<int>(std::floor(exit_pt[0]));
                y = static_cast<int>(std::floor(exit_pt[1]));

                // ========== 增量式更新全局 tMaxX、tMaxY ==========
                float delta_tx, delta_ty;
                if (dir[0] > 0.f)
                    delta_tx = (static_cast<float>(x + 1) - exit_pt[0]) * tDeltaX;
                else
                    delta_tx = (exit_pt[0] - static_cast<float>(x)) * tDeltaX;

                if (dir[1] > 0.f)
                    delta_ty = (static_cast<float>(y + 1) - exit_pt[1]) * tDeltaY;
                else
                    delta_ty = (exit_pt[1] - static_cast<float>(y)) * tDeltaY;

                tMaxX = t_block_exit + delta_tx;
                tMaxY = t_block_exit + delta_ty;
                // ================================================
            } else {
                // 下一步跨网格的最小t值
                float tNext = std::min(tMaxX, tMaxY);
                // 到达终点区间，直接退出，不再前进
                if (tNext >= 1.f - eps)
                    return false;

                if (tMaxX < tMaxY)
                {
                    tMaxX += tDeltaX;
                    x += stepX;
                }
                else
                {
                    tMaxY += tDeltaY;
                    y += stepY;
                }
            }
        }

        return false;
    }



template<typename T>
bool lineOfSightCheckAW(const Point<T,2>& start, const Point<T, 2>& end,
                        int gridW, int gridH, const IS_OCCUPIED_FUNC<2>& isoc,
                        JOB::BlockDetectorInterfacePtr<2> block_detector_ptr) {

    const float eps = 1e-6f;
    Point<T,2> dir{end[0] - start[0], end[1] - start[1]};

    Pointi<2> current_pt;
    Pointf<2> current_fpt;
    Id current_id;

    // 1. 起点终点完全重合
    if (std::fabs(dir[0]) < eps && std::fabs(dir[1]) < eps)
    {
        //std::cout << "case 0 " << std::endl;
        int x0 = static_cast<int>(std::floor(start[0]));
        int y0 = static_cast<int>(std::floor(start[1]));
        if (x0 >= 0 && x0 < gridW && y0 >= 0 && y0 < gridH) {
            if(isoc(Pointi<2>{x0, y0})) {
                return true;
            }
        }
        return false;
    }

    // 2. 纯垂直线：x全程不变，只遍历y轴
    if (std::fabs(dir[0]) < eps) {
        //std::cout << "case 1 " << std::endl;
        int fixedX = static_cast<int>(std::floor(start[0]));
        float yMinF = std::min(start[1], end[1]);
        float yMaxF = std::max(start[1], end[1]);
        int yStart = static_cast<int>(std::floor(yMinF));
        int yEnd = static_cast<int>(std::floor(yMaxF));

        // for循环遍历所有y栅格
        for (int y = yStart; y <= yEnd; ++y) {
            // 栅格边界校验
            if (fixedX >= 0 && fixedX < gridW && y >= 0 && y < gridH)
            {
                current_pt = Pointi<2>{fixedX, y};
                if(isoc(Pointi<2>{fixedX, y})) {
                    return true;
                }
                current_id = PointiToId(current_pt, block_detector_ptr->dimension_info_);
                BlockPtr<2> current_block = block_detector_ptr->block_ptr_map_[current_id];
                if(current_block != nullptr) {
                    if (current_block->max_[1]+1 > yEnd) {
                        return false;
                    } else {
                        y = current_block->max_[1];
                    }
                }
            } else {
                return false;
            }
        }
        return false;
    }

    // 3. 纯水平线：y全程不变，只遍历x轴
    if (std::fabs(dir[1]) < eps) {
        //std::cout << "case 2 " << std::endl;
        int fixedY = static_cast<int>(std::floor(start[1]));
        float xMinF = std::min(start[0], end[0]);
        float xMaxF = std::max(start[0], end[0]);
        int xStart = static_cast<int>(std::floor(xMinF));
        int xEnd = static_cast<int>(std::floor(xMaxF));

        // for循环遍历所有x栅格
        for (int x = xStart; x <= xEnd; ++x)
        {
            // 栅格边界校验
            if (x >= 0 && x < gridW && fixedY >= 0 && fixedY < gridH)
            {
                current_pt = Pointi<2>{x, fixedY};
                if(isoc(Pointi<2>{x, fixedY})) {
                    return true;
                }
                current_id = PointiToId(current_pt, block_detector_ptr->dimension_info_);
                BlockPtr<2> current_block = block_detector_ptr->block_ptr_map_[current_id];
                if(current_block != nullptr) {
                    if (current_block->max_[0]+1 > xEnd) {
                        return false;
                    } else {
                        x = current_block->max_[0];
                    }
                }
            }
        }
        return false;
    }
    //std::cout << "case 3 " << std::endl;

    // 当为斜线时
    // 当前栅格
    int x = static_cast<int>(std::floor(start[0]));
    int y = static_cast<int>(std::floor(start[1]));

    // 步进方向 ±1
    int stepX = dir[0] > 0.f ? 1 : -1;
    int stepY = dir[1] > 0.f ? 1 : -1;

    // tDelta：沿轴移动1格对应的t增量（t是沿射线的参数，0=起点，1=终点）
    float tDeltaX = std::fabs(1.f / dir[0]);
    float tDeltaY = std::fabs(1.f / dir[1]);

    // tMax：首次穿过网格线的t值
    float tMaxX, tMaxY;
    if (dir[0] > 0.f)
        tMaxX = (x + 1.f - start[0]) * tDeltaX;
    else
        tMaxX = (start[0] - x) * tDeltaX;

    if (dir[1] > 0.f)
        tMaxY = (y + 1.f - start[1]) * tDeltaY;
    else
        tMaxY = (start[1] - y) * tDeltaY;

    float pre_tMaxX = 1e20, pre_tMaxY = 1e20;
    bool step_by_step = false;
    while (x >= 0 && x < gridW && y >= 0 && y < gridH) {
        //std::cout << "tMaxX/Y = " << tMaxX << "/" << tMaxY << std::endl;
        step_by_step = false;
        if(pre_tMaxX == tMaxX && pre_tMaxY == tMaxY) {
            step_by_step = true;
        }
        pre_tMaxX = tMaxX; pre_tMaxY = tMaxY;
        current_pt = Pointi<2>{x, y};
        if(tMaxX < tMaxY) {
            current_fpt = Pointf<2>{start[0] + tMaxX*dir[0], start[1] + tMaxX*dir[1]};
        } else {
            current_fpt = Pointf<2>{start[0] + tMaxY*dir[0], start[1] + tMaxY*dir[1]};
        }

        if(isoc(current_pt)) {
            return true;
        }
        current_id = PointiToId(current_pt, block_detector_ptr->dimension_info_);
        BlockPtr<2> current_block = block_detector_ptr->block_ptr_map_[current_id];
        if (!step_by_step && current_block != nullptr) {
            Pointi<2> block_min = current_block->min_;
            Pointi<2> block_max = current_block->max_+Pointi<2>{1,1}; // the real boundary of current block

            //std::cout << "block_max/min = " << block_max << "/" << block_min << std::endl;

            float t_candidate_x = 1e20f;
            float t_candidate_y = 1e20f;

            if(std::fabs(dir[0]) > eps) {
                float b_min_x = static_cast<float>(block_min[0]);
                float b_max_x = static_cast<float>(block_max[0]) + 1.0f;
                float t_x_low  = (b_min_x - start[0]) / dir[0];
                float t_x_high = (b_max_x - start[0]) / dir[0];
                if(dir[0] > 0)
                    t_candidate_x = t_x_high;
                else
                    t_candidate_x = t_x_low;
            }

            if(std::fabs(dir[1]) > eps) {
                float b_min_y = static_cast<float>(block_min[1]);
                float b_max_y = static_cast<float>(block_max[1]) + 1.0f;
                float t_y_low  = (b_min_y - start[1]) / dir[1];
                float t_y_high = (b_max_y - start[1]) / dir[1];
                if(dir[1] > 0)
                    t_candidate_y = t_y_high;
                else
                    t_candidate_y = t_y_low;
            }

            float t_block_exit = std::min(t_candidate_x, t_candidate_y);

            //std::cout << "t_block_exit = " << t_block_exit << std::endl;

            if(t_block_exit >= 1.0f - eps) {
                return false;
            }

            Pointf<2> exit_pt{
                    start[0] + t_block_exit * dir[0],
                    start[1] + t_block_exit * dir[1]
            };

            // 更新跳跃后栅格索引
            x = static_cast<int>(std::floor(exit_pt[0]));
            y = static_cast<int>(std::floor(exit_pt[1]));

            // ========== 增量式更新全局 tMaxX、tMaxY ==========
            float delta_tx, delta_ty;
            if (dir[0] > 0.f)
                delta_tx = (static_cast<float>(x + 1) - exit_pt[0]) * tDeltaX;
            else
                delta_tx = (exit_pt[0] - static_cast<float>(x)) * tDeltaX;

            if (dir[1] > 0.f)
                delta_ty = (static_cast<float>(y + 1) - exit_pt[1]) * tDeltaY;
            else
                delta_ty = (exit_pt[1] - static_cast<float>(y)) * tDeltaY;

            tMaxX = t_block_exit + delta_tx;
            tMaxY = t_block_exit + delta_ty;
            // ================================================
        } else {
            // 下一步跨网格的最小t值
            float tNext = std::min(tMaxX, tMaxY);
            // 到达终点区间，直接退出，不再前进
            if (tNext >= 1.f - eps)
            return false;

            if (tMaxX < tMaxY) {
                tMaxX += tDeltaX;
                x += stepX;
            } else {
                tMaxY += tDeltaY;
                y += stepY;
            }
        }
    }
    return false;
}

}

#endif //FREENAV_LOS_JUMP_BETWEEN_BLOCK_H
