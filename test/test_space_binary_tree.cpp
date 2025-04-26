//
// Created by yaozhuo on 2025/4/24.
//

#include "gtest/gtest.h"
#include "octomap/octomap.h"
#include "../test/test_data.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/visualization/canvas/canvas.h"

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

DimensionLength dim[2];

TEST(getIndex, test) {

    dim[0] = 13, dim[1] = 11;

    auto is_occupied = [](const Pointi<2> & pt) -> bool {
        if(pt[0] >= dim[0] || pt[0] < 0) {
            return true;
        }
        if(pt[1] >= dim[1] || pt[1] < 0) {
            return true;
        }
        return false;
    };

    SpaceBinaryTree<2> sbt(is_occupied, dim);
    sbt.printTree();

    Id total_index = getTotalIndexOfSpace<2>(dim);
    for (Id id = 0; id < total_index; id++) {
        Pointi<2> pt = IdToPointi<2>(id, dim);
        std::cout << pt << ": ";
        for(int depth=0; depth<sbt.max_depth_; depth++) {
            std::cout << sbt.getIndex(pt, depth) << " ";
        }
        std::cout << std::endl;
    }
}

TEST(setOccupiedState, test) {

    dim[0] = 13, dim[1] = 11;

    auto is_occupied = [](const Pointi<2> & pt) -> bool {
        if(pt[0] >= dim[0] || pt[0] < 0) {
            return true;
        }
        if(pt[1] >= dim[1] || pt[1] < 0) {
            return true;
        }
        return false;
    };

    SpaceBinaryTree<2> sbt(is_occupied, dim);
    Pointi<2> pt = Pointi<2>{0, 0};

    sbt.setOccupiedState(pt, false);
    //sbt.printTree();
    std::cout << pt << " state = " << sbt.isOccupied(pt) << std::endl;
    sbt.setOccupiedState(pt, true);
    //sbt.printTree();
    std::cout << pt << " state = " << sbt.isOccupied(pt) << std::endl;

    pt = Pointi<2>{0, 1};
    sbt.setOccupiedState(pt, false);
    std::cout << pt << " state = " << sbt.isOccupied(pt) << std::endl;
    sbt.setOccupiedState(Pointi<2>{0, 1}, true);
    std::cout << pt << " state = " << sbt.isOccupied(pt) << std::endl;

    pt = Pointi<2>{0, 0};
    std::cout << pt << " state = " << sbt.isOccupied(pt) << std::endl;

}

auto map_test_config = MapTestConfig_Shanghai_0_512;
std::string vis_file_path    = map_test_config.at("vis_path");

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};


TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
int zoom_rate = 1;

TEST(SpaceBinaryTree, test) {
    auto dimension = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    SpaceBinaryTree<2> sbt(is_occupied, dimension);

    Id total_index = getTotalIndexOfSpace<2>(dimension);

    for(Id id=0; id<total_index; id++) {
        Pointi<2> pt = IdToPointi<2>(id, dimension);
        assert(is_occupied(pt) == sbt.isOccupied(pt));
    }

//    Canvas canvas("SpaceBinaryTree", dimension[0], dimension[1], .05, zoom_rate);
//
//    while(1) {
//        canvas.resetCanvas();
//        canvas.drawEmptyGrid();
//        canvas.drawGridMap(dimension, is_occupied_func);
//
//        //canvas.drawGridMap(down_sampled_map.dimension_infos_.back(), is_occupied_downsample_func);
//
//        char key = canvas.show(300);
//        switch (key) {
//            case 'w':
//                break;
//            case 's':
//                break;
//            default:
//                break;
//        }
//        //break;
//    }
}