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

#include "../algorithm/space_binary_tree/space_binary_tree_raw.h"
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

    SpaceBinaryTree2DRaw sbt(is_occupied, dim);
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

//TEST(setOccupiedState, test) {
int main() {

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

//    SpaceBinaryTreeAnyDimensionRaw<2> sbt(is_occupied, dim);
    SpaceBinaryTreeAnyDimension<2> sbt(is_occupied, dim);
    sbt.initialize();
    sbt.printTree();


    Pointi<2> pt = Pointi<2>{7, 7};

    sbt.setOccupiedState(pt, true);
    sbt.globalRecursiveUpdate();
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

// MapTestConfig_Shanghai_0_512
auto map_test_config = MapTestConfig_Shanghai_0_512;
std::string vis_file_path    = map_test_config.at("vis_path");

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};


TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
int zoom_rate = 1;

TEST(SpaceBinaryTree2D, test) {
//int main() {
    auto dimension = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

    gettimeofday(&tv_pre, &tz);

//    SpaceBinaryTreeAnyDimensionRaw<2> sbt(is_occupied, dimension);
    SpaceBinaryTreeAnyDimension<2> sbt(is_occupied, dimension);
    sbt.initialize();
    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree2D take " << time_cost << " ms to initialize" << std::endl;

    Id total_index = getTotalIndexOfSpace<2>(dimension);

    for(Id id=0; id<total_index; id++) {
        Pointi<2> pt = IdToPointi<2>(id, dimension);
        assert(is_occupied(pt) == sbt.isOccupied(pt));
    }
    //return 0;
    std::vector<TreeNodeNewPtr<2> > free_leaf_nodes = sbt.getAllPassableLeafNodes();
    //std::vector<TreeNodePtr<2> > free_leaf_nodes = sbt.getAllPassableLeafNodes();

    BlockPtrs<2> block_ptrs;
    for(const auto& leaf_node : free_leaf_nodes) {
//        std::cout << "leaf_node = " << leaf_node << std::endl;
        BlockPtr<2> block_ptr = std::make_shared<Block<2> >();
//        std::cout << "leaf node depth_ = " << leaf_node->depth_ << std::endl;
//        std::cout << "leaf_node->base_pt_ = " << leaf_node->base_pt_ << std::endl;
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
    //return 0;
    Canvas canvas("SpaceBinaryTree2D",dimension[0],dimension[1], .05, zoom_rate);
    bool draw_free_leaf = true,
         draw_block = false;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied);

        if(draw_free_leaf) {
            //canvas.draw_DistMap(block_detect.dimension_info_, block_detect.dist_map_);
            //int total_count = getTotalIndexOfSpace<2>(dimension);
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


// MapTestConfig_Complex memory 1.5G
// MapTestConfig_DA1
auto map_test_config_3D = MapTestConfig_Complex; // 3780.27

TextMapLoader_3D loader3D(map_test_config_3D.at("map_path"));


TEST(SpaceBinaryTree3D, test) {
//int main() {
    auto dimension = loader3D.getDimensionInfo();

    auto is_occupied = [](const Pointi<3> & pt) -> bool { return loader3D.isOccupied(pt); };

    std::cout << "SpaceBinaryTree3D start initialize" << std::endl;


    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTreeAnyDimensionRaw<3> sbt_raw(is_occupied, dimension);
    sbt_raw.initialize();

    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree3D RAW take " << time_cost << " ms to initialize" << std::endl;

    Id total_index = getTotalIndexOfSpace<3>(dimension);

    for(Id id=0; id<total_index; id++) {
        Pointi<3> pt = IdToPointi<3>(id, dimension);
        assert(is_occupied(pt) == sbt_raw.isOccupied(pt));
    }

    std::cout << "SpaceBinaryTree3D RAW num of passable leaf = " << sbt_raw.getAllPassableLeafNodes().size() << std::endl;
    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTreeAnyDimension<3> sbt(is_occupied, dimension);
    sbt.initialize();

    gettimeofday(&tv_after, &tz);

    time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "SpaceBinaryTree3D take " << time_cost << " ms to initialize" << std::endl;

    total_index = getTotalIndexOfSpace<3>(dimension);

    for(Id id=0; id<total_index; id++) {
        Pointi<3> pt = IdToPointi<3>(id, dimension);
        assert(is_occupied(pt) == sbt.isOccupied(pt));
    }
    std::cout << "SpaceBinaryTree3D num of passable leaf = " << sbt.getAllPassableLeafNodes().size() << std::endl;

}

TEST(GetFloorOrCeilFlag, test) {
    Pointis<2> offsets = GetFloorOrCeilFlag<2>();
    for(const auto& offset : offsets) {
        std::cout << offset << std::endl;
    }
}

//         sbt = std::make_shared<SpaceBinaryTree2D>(isoc_temp, temp_dim, 3);


void LineOfSightTest2D(DimensionLength* temp_dim, const IS_OCCUPIED_FUNC<2>& isoc_temp, int min_block_depth_width = 4) {

    gettimeofday(&tv_pre, &tz);

    SpaceBinaryTreePtr<2> sbt = std::make_shared<SpaceBinaryTree2DRaw>(isoc_temp, temp_dim, min_block_depth_width);

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

    SpaceBinaryTreePtr<N> sbt = std::make_shared<SpaceBinaryTreeAnyDimensionRaw<N> >(isoc_temp, temp_dim, 3);
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

TEST(LineOfSightCheck2D, test) {
//int main() {
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