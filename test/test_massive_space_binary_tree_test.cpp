//
// Created by yaozhuo on 2025/4/29.
//

#include "gtest/gtest.h"
#include "../algorithm/space_binary_tree/space_binary_tree.h"
#include "dynamic_obstacles.h"
#include "../freeNav-base/visualization/canvas/canvas.h"
#include "dependencies.h"


// dynamic map
// time cost of update dynamic map
// time cost of raw LOS check and SpaceBinaryTree's LOS check

using namespace freeNav::JOB;
using namespace freeNav;



TEST(dynamic_obstacles_2D, test) {
    DimensionLength dim[2];

    dim[0] = 130, dim[1] = 110;

    auto is_occupied = [&](const Pointi<2> & pt) -> bool {
        if(pt[0] >= dim[0] || pt[0] < 0) {
            return true;
        }
        if(pt[1] >= dim[1] || pt[1] < 0) {
            return true;
        }
        return false;
    };

    ObstaclePtrs<2> obs = {
            std::make_shared<CircleObstacle<2> >(30),
            std::make_shared<BlockObstacle<2> >(Pointi<2>{30, 20}),
            };

    CircleObstaclePtrs<2> co = generateRandomCircleObstacles<2>(5, 2, 5);
    obs.insert(obs.end(), co.begin(), co.end());

    BlockObstaclePtrs<2> bo = generateRandomBlockObstacles<2>(5, Pointi<2>{2, 2}, Pointi<2>{5, 5});
    obs.insert(obs.end(), co.begin(), co.end());

    DynamicObstacles<2> dynamic_obstacles(dim, obs);

    //dynamic_obstacles.random();

    SpaceBinaryTree<2> sbt(is_occupied, dim);


    Canvas canvas("dynamic_obstacles_2D", dim[0], dim[1], .05, 1000/std::max(dim[0], dim[1]));

    bool draw_pre_occupy = false,
         draw_current_occupy = true,
         draw_free_leaf = false,
         draw_block = true;

    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
        if(draw_pre_occupy) {
            canvas.drawGrids(dynamic_obstacles.getPreviousOccupationPoints());
            canvas.drawGrids(dynamic_obstacles.previous_center_pts_, COLOR_TABLE[1]);
        }
        if(draw_current_occupy) {
            canvas.drawGrids(dynamic_obstacles.getCurrentOccupationPoints());
            canvas.drawGrids(dynamic_obstacles.current_center_pts_, COLOR_TABLE[0]);
        }
        if(draw_block) {
            for(int id=0; id<sbt.block_ptr_map_.size(); id++) {
                if(sbt.block_ptr_map_[id] == nullptr) { continue; }
                Pointi<2> pt = IdToPointi<2>(id, dim);
                Id indicator = PointiToId<2>(sbt.block_ptr_map_[id]->min_, dim);
                canvas.drawGrid(pt[0], pt[1], COLOR_TABLE[indicator%30]);
            }
        }
        if(draw_free_leaf) {
            std::vector<TreeNodePtr<2> > free_leaf_nodes = sbt.getAllPassableLeafNodes();
            BlockPtrs<2> block_ptrs;
            for(const auto& leaf_node : free_leaf_nodes) {
                BlockPtr<2> block_ptr = std::make_shared<Block<2> >();
                block_ptr->min_ = leaf_node->base_pt_;
                Pointi<2> offset; offset.setAll(sbt.pow_2_[sbt.max_depth_-leaf_node->depth_]-1);
                block_ptr->max_ = leaf_node->base_pt_ + offset;
                assert((!is_occupied(block_ptr->min_)) && (!is_occupied(block_ptr->max_)));
                block_ptrs.push_back(block_ptr);
            }
            for(int i=0; i<block_ptrs.size(); i++) {
                const auto& block_ptr = block_ptrs[i];
                const Pointi<2> pt1 = block_ptr->min_, pt2 = block_ptr->max_;
                //const Pointi<2> pt1 = Pointi<2>{0, 0}, pt2 = Pointi<2>{100, 100};
                canvas.drawGridLine(pt1[0], pt1[1], pt1[0], pt2[1], 1, false,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt1[0], pt2[1], pt2[0], pt2[1], 1, false,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt2[1], pt2[0], pt1[1], 1, false,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt1[1], pt1[0], pt1[1], 1, false,COLOR_TABLE[i%30]);
                //break;
            }
        }
        char key = canvas.show(30);
        switch (key) {
            case 'p':
                draw_pre_occupy = !draw_pre_occupy;
                break;
            case 'c':
                draw_current_occupy = !draw_current_occupy;
                break;
            case 32: // 32 means space
                dynamic_obstacles.random();
                for(const auto& pre_pt : dynamic_obstacles.getPreviousOccupationPoints()) {
                    sbt.setOccupiedState(pre_pt, false);
                }
                for(const auto& cur_pt : dynamic_obstacles.getCurrentOccupationPoints()) {
                    sbt.setOccupiedState(cur_pt, true);
                }
                break;
            case 'f':
                draw_free_leaf = !draw_free_leaf;
                break;
            case 'b':
                draw_block = !draw_block;
                break;
            default:
                break;
        }
    }

}

TEST(massiveSBTLOSCompareTest, test) {

    massiveSBTLOSCompareTest<2>(10, 100, {200,300,400}, {10,20,40});

}