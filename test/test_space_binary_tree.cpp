//
// Created by yaozhuo on 2025/4/24.
//

#include "gtest/gtest.h"
#include "octomap/octomap.h"
#include "../test/test_data.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"

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
#include "dependencies.h"

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

    SpaceBinaryTree2D sbt(is_occupied, dim);
    sbt.initialize();
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

    dim[0] = 8, dim[1] = 8;

    auto is_occupied = [](const Pointi<2> & pt) -> bool {
        if(pt[0] >= dim[0] || pt[0] < 0) {
            return true;
        }
        if(pt[1] >= dim[1] || pt[1] < 0) {
            return true;
        }
        return false;
    };

    SpaceBinaryTreeAnyDimension<2> sbt(is_occupied, dim);
    sbt.initialize();
    sbt.printTree();


    Pointi<2> pt = Pointi<2>{7, 7};

    sbt.setOccupiedState(pt, true);
    sbt.printTree();
//    std::cout << pt << " state = " << sbt.isOccupied(pt) << std::endl;
//
//    sbt.setOccupiedState(pt, false);
//    sbt.printTree();
//    std::cout << pt << " state = " << sbt.isOccupied(pt) << std::endl;
//
//    Id total_index = getTotalIndexOfSpace<2>(dim);
//    for(Id id=0; id<total_index; id++) {
//        Pointi<2> pt = IdToPointi<2>(id, dim);
//        sbt.setOccupiedState(pt, true);
//    }
//    sbt.printTree();
//
//    for(Id id=0; id<total_index; id++) {
//        Pointi<2> pt = IdToPointi<2>(id, dim);
//        sbt.setOccupiedState(pt, false);
//    }
//    sbt.printTree();

}

auto map_test_config = MapTestConfig_Shanghai_0_512;
std::string vis_file_path    = map_test_config.at("vis_path");

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};


TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
int zoom_rate = 1;

//MapTestConfig_Complex 7796.59 ms 4277.9 ms
//MapTestConfig_A1 109404 ms
// A1 10000 LOS test, mean raw LOS time cost = 0.0077252, mean SBT LOS time cost = 0.0059969
// Complex 1000000 LOS test, mean raw LOS time cost = 0.00199108, mean SBT LOS time cost = 0.00311197
// MapTestConfig_Complex
auto map_test_config_3D = MapTestConfig_A1;

TextMapLoader_3D loader3D(map_test_config_3D.at("map_path"));


TEST(GetFloorOrCeilFlag, test) {
    Pointis<2> offsets = GetFloorOrCeilFlag<2>();
    for(const auto& offset : offsets) {
        std::cout << offset << std::endl;
    }
}

//         sbt = std::make_shared<SpaceBinaryTree2D>(isoc_temp, temp_dim, 3);


void LineOfSightTest2D(DimensionLength* temp_dim, const IS_OCCUPIED_FUNC<2>& isoc_temp, int min_block_depth_width = 4) {

    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTreePtr<2> sbt = std::make_shared<SpaceBinaryTree2D>(isoc_temp, temp_dim, min_block_depth_width);

    sbt->initialize();

    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree" << 2 << "D, min_block_depth = " << sbt->min_block_depth_width_
              << " take " << time_cost << " ms to initialize" << std::endl;

    Id total_index = getTotalIndexOfSpace<2>(temp_dim);
    srand(time(0));

    for(Id id=0; id<total_index; id++) {
        Pointi<2> pt = IdToPointi<2>(id, temp_dim);
        assert(isoc_temp(pt) == sbt->isOccupied(pt));
    }

    LOSCompare<2>(temp_dim, isoc_temp, sbt, 1e6, 1e3);

    std::cout << "raw / SBT visited pt = " << sbt->raw_visited_pt_count_ << " / " << sbt->SBT_visited_pt_count_ << std::endl;
}

void LineOfSightTest3D(DimensionLength* temp_dim, const IS_OCCUPIED_FUNC<3>& isoc_temp, int min_block_depth_width = 4) {

    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTreePtr<3> sbt = std::make_shared<SpaceBinaryTree3D>(isoc_temp, temp_dim, min_block_depth_width);

    sbt->initialize();

    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree" << 3 << "D, min_block_depth = " << sbt->min_block_depth_width_
              << " take " << time_cost << " ms to initialize" << std::endl;

    Id total_index = getTotalIndexOfSpace<3>(temp_dim);
    srand(time(0));

    for(Id id=0; id<total_index; id++) {
        Pointi<3> pt = IdToPointi<3>(id, temp_dim);
        assert(isoc_temp(pt) == sbt->isOccupied(pt));
    }

    LOSCompare<3>(temp_dim, isoc_temp, sbt, 1e6, 1e3);

    std::cout << "raw / SBT visited pt = " << sbt->raw_visited_pt_count_ << " / " << sbt->SBT_visited_pt_count_ << std::endl;
}

template<Dimension N>
void LineOfSightTest(DimensionLength* temp_dim, const IS_OCCUPIED_FUNC<N>& isoc_temp) {

    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTreePtr<N> sbt = std::make_shared<SpaceBinaryTreeAnyDimension<N> >(isoc_temp, temp_dim, 3);
    sbt->initialize();

    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree" << N << "D, min_block_depth = " << sbt->min_block_depth_width_
              << " take " << time_cost << " ms to initialize" << std::endl;

    Id total_index = getTotalIndexOfSpace<N>(temp_dim);
    Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();
    srand(time(0));

    for(Id id=0; id<total_index; id++) {
        Pointi<N> pt = IdToPointi<N>(id, temp_dim);
        assert(isoc_temp(pt) == sbt->isOccupied(pt));
    }

    LOSCompare<N>(temp_dim, isoc_temp, sbt);

    std::cout << "raw / SBT visited pt = " << sbt->raw_visited_pt_count_ << " / " << sbt->SBT_visited_pt_count_ << std::endl;
}

// MapTestConfig_Shanghai_0_512
// 1000000 LOS test, mean raw LOS time cost = 0.00166777, mean SBT LOS time cost = 0.00124032

//TEST(LineOfSightCheck2D, test) {
int main() {
    auto dimension = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

    //LineOfSightTest<2>(dimension, is_occupied);
    for(int i=2; i<=2; i++) {
        LineOfSightTest2D(dimension, is_occupied, i);
    }
}


TEST(LineOfSightCheck3D, test) {
//int main() {
    auto dimension = loader3D.getDimensionInfo();

    auto is_occupied = [](const Pointi<3> & pt) -> bool { return loader3D.isOccupied(pt); };

    for(int i=4; i<=4; i++)
    {
        LineOfSightTest3D(dimension, is_occupied, i);
    }
}