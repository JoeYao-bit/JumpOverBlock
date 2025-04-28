//
// Created by yaozhuo on 2025/4/29.
//

#include "gtest/gtest.h"
#include "../algorithm/space_binary_tree/space_binary_tree.h"
#include "dynamic_obstacles.h"
#include "../freeNav-base/visualization/canvas/canvas.h"


// dynamic map
// time cost of update dynamic map
// time cost of raw LOS check and SpaceBinaryTree's LOS check

using namespace freeNav::JOB;
using namespace freeNav;

DimensionLength dim[2];


TEST(dynamic_obstacles_2D, test) {

    dim[0] = 130, dim[1] = 110;

    auto is_occupied = [](const Pointi<2> & pt) -> bool {
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

    DynamicObstacles<2> dynamic_obstacles(dim, obs);

    dynamic_obstacles.random();

    Canvas canvas("dynamic_obstacles_2D", dim[0], dim[1], .05, 1000/std::max(dim[0], dim[1]));

    bool draw_pre_occupy = false,
         draw_current_occupy = true;

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
                break;
            default:
                break;
        }
    }

}