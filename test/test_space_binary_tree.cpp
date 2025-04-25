//
// Created by yaozhuo on 2025/4/24.
//

#include "gtest/gtest.h"
#include "octomap/octomap.h"

//#include "../freeNav-base/visualization/canvas/canvas.h"
//#include "../test/test_data.h"
//#include "../freeNav-base/dependencies/random_map_generator.h"
//#include "../freeNav-base/basic_elements/distance_map_update.h"
//#include "../freeNav-base/basic_elements/map_down_sampler.h"
//#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
//#include "../freeNav-base/dependencies/thread_pool.h"
//
//#include "../algorithm/block_detect.h"
//#include "../algorithm/block_detector_greedy.h"
//#include "../algorithm/line_of_sight_jump_between_block.h"

#include "../algorithm/space_binary_tree/space_binary_tree.h"

using namespace freeNav::JOB;
using namespace freeNav;


TEST(space_binary_tree, test) {

    DimensionLength dim[2];
    dim[0] = 3, dim[1] = 5;

    auto is_occupied = [](const Pointi<2> & pt) -> bool {
        return true;
    };

    SpaceBinaryTree<2> sbt(is_occupied, dim);

}