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

    SpaceBinaryTree<2> sbt(is_occupied, dim);

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

    // set to reverse state
//    for(Id id=0; id<total_index; id++) {
//        Pointi<2> pt = IdToPointi<2>(id, dimension);
//        sbt.setOccupiedState(pt, !is_occupied(pt));
//    }

    Canvas canvas("SpaceBinaryTree2D",dimension[0],dimension[1], .05, zoom_rate);
    bool draw_free_leaf = true,
         draw_block = false;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied);

        if(draw_free_leaf) {
            //canvas.draw_DistMap(block_detect.dimension_info_, block_detect.dist_map_);
            int total_count = getTotalIndexOfSpace<2>(dimension);
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
        if(draw_block) {
            for(int id=0; id<sbt.block_ptr_map_.size(); id++) {
                if(sbt.block_ptr_map_[id] == nullptr) { continue; }
                Pointi<2> pt = IdToPointi<2>(id, dimension);
                Id indicator = PointiToId<2>(sbt.block_ptr_map_[id]->min_, dimension);
                canvas.drawGrid(pt[0], pt[1], COLOR_TABLE[indicator%30]);
            }
        }
        char key = canvas.show(30);
        switch (key) {
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

//MapTestConfig_Complex 7796.59 ms 4277.9 ms
//MapTestConfig_A1 109404 ms
// A1 10000 LOS test, mean raw LOS time cost = 0.0077252, mean SBT LOS time cost = 0.0059969
// Complex 1000000 LOS test, mean raw LOS time cost = 0.00199108, mean SBT LOS time cost = 0.00311197
auto map_test_config_3D = MapTestConfig_Complex;

TextMapLoader_3D loader3D(map_test_config_3D.at("map_path"));


TEST(SpaceBinaryTree3D, test) {
    auto dimension = loader3D.getDimensionInfo();

    auto is_occupied = [](const Pointi<3> & pt) -> bool { return loader3D.isOccupied(pt); };

    std::cout << "SpaceBinaryTree3D start initialize" << std::endl;


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

TEST(GetFloorOrCeilFlag, test) {
    Pointis<2> offsets = GetFloorOrCeilFlag<2>();
    for(const auto& offset : offsets) {
        std::cout << offset << std::endl;
    }
}

template<Dimension N>
void LineOfSightTest(DimensionLength* temp_dim, const IS_OCCUPIED_FUNC<N>& isoc_temp) {

    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTree<N> sbt(isoc_temp, temp_dim);

    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree" << N << "D take " << time_cost << " ms to initialize" << std::endl;

    Id total_index = getTotalIndexOfSpace<N>(temp_dim);
    Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();
    srand(time(0));

    for(Id id=0; id<total_index; id++) {
        Pointi<N> pt = IdToPointi<N>(id, temp_dim);
        assert(isoc_temp(pt) == sbt.isOccupied(pt));
    }

    double sum_1 = 0, sum_2 = 0;
    int success_count = 0;
    for(int i=0; i<1e6; i++) {
        // random pick two passable point
        Id id1 = 0, id2 = 0;
        Pointi<N> pt1, pt2;
        int count = 1000;
        while(count >= 0) {
            id1 = rand() % total_index;
            pt1 = IdToPointi<N>(id1, temp_dim);
            if (!isoc_temp(pt1)) {
                break;
            } else {
                count --;
            }
        }
        if (isoc_temp(pt1)) {
            continue;
        }
        count = 1000;
        while(count >= 0) {
            id2 = rand() % total_index;
            pt2 = IdToPointi<N>(id2, temp_dim);
            if (!isoc_temp(pt2)) {
                break;
            } else {
                count --;
            }
        }
        if (isoc_temp(pt2)) {
            continue;
        }
        //std::cout << "do LOS between " << pt1 << ", " << pt2 <<  std::endl;
        gettimeofday(&tv_pre, &tz);
        bool isoc1 = LineCrossObstacle<N>(pt1, pt2, isoc_temp, neighbor);
        gettimeofday(&tv_after, &tz);
        double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        sum_1 = sum_1 + time_cost1;
        Pointis<N> visited_pt;
        int count_of_block;
        gettimeofday(&tv_pre, &tz);
        bool isoc2 = sbt.lineCrossObstacle(pt1, pt2, visited_pt, count_of_block);
        gettimeofday(&tv_after, &tz);
        double time_cost2 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        sum_2 = sum_2 + time_cost2;
        success_count ++;
        if(isoc1 != isoc2) {
            std::cout << i << " th test failed, pt1/pt2 = " << pt1 << " / " << pt2 << std::endl;
            // assert classic LOS check and SBT's LOS check have the same result
            assert(isoc1 == isoc2);
        }
    }

    std::cout << success_count <<  " LOS test, mean raw LOS time cost = " << sum_1/(double)success_count
              << ", mean SBT LOS time cost = " << sum_2/(double)success_count << std::endl;

}

// MapTestConfig_Shanghai_0_512
// 1000000 LOS test, mean raw LOS time cost = 0.00166777, mean SBT LOS time cost = 0.00124032

TEST(LineOfSightCheck2D, test) {
    auto dimension = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

    LineOfSightTest<2>(dimension, is_occupied);
}


TEST(LineOfSightCheck3D, test) {
    auto dimension = loader3D.getDimensionInfo();

    auto is_occupied = [](const Pointi<3> & pt) -> bool { return loader3D.isOccupied(pt); };

    LineOfSightTest<3>(dimension, is_occupied);

}