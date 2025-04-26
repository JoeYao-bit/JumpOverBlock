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
#include "../freeNav-base/dependencies/3d_textmap/voxel_loader.h"

using namespace freeNav::JOB;
using namespace freeNav;

DimensionLength dim[2];

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

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

TEST(SpaceBinaryTree2D, test) {
    auto dimension = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTree<2> sbt(is_occupied, dimension);

    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree2D take " << time_cost << " ms to initialize" << std::endl;

    Id total_index = getTotalIndexOfSpace<2>(dimension);

    for(Id id=0; id<total_index; id++) {
        Pointi<2> pt = IdToPointi<2>(id, dimension);
        assert(is_occupied(pt) == sbt.isOccupied(pt));
    }
}

auto map_test_config_3D = MapTestConfig_Complex;

TextMapLoader_3D loader3D(map_test_config_3D.at("map_path"));


TEST(SpaceBinaryTree3D, test) {
    auto dimension = loader3D.getDimensionInfo();

    auto is_occupied = [](const Pointi<3> & pt) -> bool { return loader3D.isOccupied(pt); };

    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTree<3> sbt(is_occupied, dimension);

    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree3D take " << time_cost << " ms to initialize" << std::endl;

    Id total_index = getTotalIndexOfSpace<3>(dimension);

    for(Id id=0; id<total_index; id++) {
        Pointi<3> pt = IdToPointi<3>(id, dimension);
        assert(is_occupied(pt) == sbt.isOccupied(pt));
    }
}
