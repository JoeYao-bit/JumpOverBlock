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
    int findExitPointOfBlock(Line<N>& line, const Pointi<N>& current_pt, int& index, const BlockPtr<N>& block_ptr) {
        // check whether the line reach end of line
        if(index >= line.step - 1) {return 0; }
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
                    future_step = (Fraction(block_ptr->max_[dim] - current_pt[dim]) /
                                   line.parameter[dim].second).toAbs();
                } else {
                    future_step = (Fraction(block_ptr->min_[dim] - current_pt[dim]) /
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
            return line.step - index - 1;
        }
        return (line.step*minimum_step_to_exit).floor();
    }

    template <Dimension N>
    bool LineCrossObstacleWithBlockJump(const Pointi<N>& pt1, const Pointi<N>& pt2,
                                        BlockDetectorInterfacePtr<N> block_detector_ptr,
                                        std::vector<Pointi<N> >& visited_pt,
                                        int& count_of_block) {
        if(pt1 == pt2) return true;
        visited_pt.clear();
        count_of_block = 0;
        Line<N> line(pt1, pt2);
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

}

#endif //FREENAV_LOS_JUMP_BETWEEN_BLOCK_H
