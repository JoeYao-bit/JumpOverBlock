//
// Created by yaozhuo on 2025/4/29.
//

#include "gtest/gtest.h"
#include "../algorithm/space_binary_tree/space_binary_tree_raw.h"
#include "../algorithm/space_binary_tree/space_binary_tree.h"
#include "../algorithm/space_binary_tree/space_binary_tree_shrink.h"

#include "dynamic_obstacles.h"
#include "../freeNav-base/visualization/canvas/canvas.h"
#include "dependencies.h"

#include <thread>
#include "test_data.h"

#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"


// dynamic map
// time cost of update dynamic map
// time cost of raw LOS check and SpaceBinaryTree's LOS check

using namespace freeNav::JOB;
using namespace freeNav;

template<Dimension N, typename SBT>
void varify_thread(DimensionLength* temp_dim,
                   const std::shared_ptr<SBT>& sbt,
                   const DynamicObstacles<N>& dynamic_obstacles,
                   const IS_OCCUPIED_FUNC<N>& isoc_temp) {
    std::vector<std::string> output_strings;
    //SpaceBinaryTreeVarify(temp_dim, isoc_temp, sbt);
    LOSCompare<N, SBT>(temp_dim, sbt, output_strings, "", 1e4);

}


auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};


// MapTestConfig_ost003d // ok
// MapTestConfig_orz900d // ok
// MapTestConfig_lak303d // ok
// MapTestConfig_den520d // ok
// MapTestConfig_den312d // ok
// MapTestConfig_Shanghai_0_512 // ok
// MapTestConfig_Simple_2D

// MapTestConfig_fr101 // ok
// MapTestConfig_edmonton // ok
// MapTestConfig_intel // ok, too small
// MapTestConfig_mexico // ok
// MapTestConfig_fhw_rec_001 // ok


auto map_test_config = MapTestConfig_Shanghai_0_512;

auto is_grid_occupied1 = [](const cv::Vec3b& color) -> bool {
    if (color != cv::Vec3b::all(255)) return true;
    return false;
};

auto is_grid_occupied2 = [](const cv::Vec3b& color) -> bool {
    if (color[0] <= 240 || color[1] <= 240 || color[2] <= 240) return true;
    return false;
};

#if 0
PictureLoader loader(map_test_config.at("map_path"), is_grid_occupied2);
#else
TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
#endif

template<typename SBT, typename TreeNode>
void dynamic_obstacles_2D() {

#if 0
    DimensionLength *dim = new DimensionLength[2];

    dim[0] = 10, dim[1] = 17;

    auto is_occupied = [&](const Pointi<2> &pt) -> bool {
        if (isOutOfBoundary<2>(pt, dim)) {
            return true;
        }
        return false;
    };
#else
    auto dim = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };
#endif

    ObstaclePtrs<2> obs = {
            std::make_shared<CircleObstacle<2> >(1),
            std::make_shared<CircleObstacle<2> >(10),
            std::make_shared<BlockObstacle<2> >(Pointi<2>{3, 5}),
            };

    CircleObstaclePtrs<2> co = generateRandomCircleObstacles<2>(2, 26, 50);
    obs.insert(obs.end(), co.begin(), co.end());

    BlockObstaclePtrs<2> bo = generateRandomBlockObstacles<2>(2, Pointi<2>{2, 2}, Pointi<2>{5, 5});
    obs.insert(obs.end(), co.begin(), co.end());

    DynamicObstacles<2> dynamic_obstacles(dim, obs);

    //dynamic_obstacles.random();
    MSTimer mst;

    std::shared_ptr<SBT> sbt = std::make_shared<SBT>(is_occupied, dim, 0);
    sbt->initialize();

    std::cout << "finish initialize of map in " << mst.elapsed() << "ms" << std::endl;


    float zoom_ratio = std::min(1800./dim[0], 1000./dim[1]);
    std::cout << "std::min(1800./dimension[0], 1000./dimension[1]) = " << zoom_ratio << std::endl;

    Canvas canvas("dynamic_obstacles_2D", dim[0], dim[1], .05, zoom_ratio);

    bool draw_pre_occupy = false,
         draw_current_occupy = true,
         draw_free_leaf = false,
         draw_block = false,
         triger_varify = false,
         draw_merged_free_leaf = true;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
        if(draw_pre_occupy) {
            //canvas.drawGrids(dynamic_obstacles.getPreviousOccupationPoints());
            Id total_index = getTotalIndexOfSpace<2>(dim);
            for(const auto& pre_id : dynamic_obstacles.pre_occ_ids_) {
                Pointi<2> pt = IdToPointi<2>(pre_id, dim);
                int x = pt[0], y = pt[1];
                canvas.drawGrid(x, y, cv::Vec3b::all(0));
            }
            canvas.drawGrids(dynamic_obstacles.previous_center_pts_, COLOR_TABLE[1]);
        }
        if(draw_current_occupy) {
//            canvas.drawGrids(dynamic_obstacles.getCurrentOccupationPoints());
            Id total_index = getTotalIndexOfSpace<2>(dim);
            for(int id=0; id<total_index; id++) {
                if(dynamic_obstacles.current_map_[id] == false) { continue; }
                Pointi<2> pt = IdToPointi<2>(id, dim);
                int x = pt[0], y = pt[1];
                canvas.drawGrid(x, y, cv::Vec3b::all(0));
            }
            canvas.drawGrids(dynamic_obstacles.current_center_pts_, COLOR_TABLE[0]);
        }
        if(draw_block) {
//            for(int id=0; id<sbt->block_ptr_map_.size(); id++) {
//                if(sbt->block_ptr_map_[id] == nullptr) { continue; }
//                Pointi<2> pt = IdToPointi<2>(id, dim);
//                Id indicator = PointiToId<2>(sbt->block_ptr_map_[id]->min_, dim);
//                canvas.drawGrid(pt[0], pt[1], COLOR_TABLE[indicator%30]);
//            }
            for(int x=0; x<dim[0]; x++) {
                for(int y=0; y<dim[1]; y++) {
                    Pointi<2> pt{x, y};
                    auto block_ptr = sbt->getInternalBlockPtr(pt);
                    if(block_ptr == nullptr) { continue; }
                    Id indicator = PointiToId<2>(block_ptr->min_, dim);
                    canvas.drawGrid(pt[0], pt[1], COLOR_TABLE[indicator%30]);
                }
            }
        }
        if(draw_free_leaf) {
            std::vector<std::shared_ptr<TreeNode> > free_leaf_nodes = sbt->getAllPassableLeafNodes();
            BlockPtrs<2> block_ptrs;
            for(const auto& leaf_node : free_leaf_nodes) {
                BlockPtr<2> block_ptr = std::make_shared<Block<2> >();
                block_ptr->min_ = leaf_node->base_pt_;
                Pointi<2> offset; offset.setAll(sbt->pow_2_[sbt->max_depth_-leaf_node->depth_]-1);
                block_ptr->max_ = leaf_node->base_pt_ + offset;
                assert((!is_occupied(block_ptr->min_)) && (!is_occupied(block_ptr->max_)));
                block_ptrs.push_back(block_ptr);
            }
            for(int i=0; i<block_ptrs.size(); i++) {
                const auto& block_ptr = block_ptrs[i];
                const Pointi<2> pt1 = block_ptr->min_, pt2 = block_ptr->max_;// + Pointi<2>{1, 1};
                canvas.drawGridLine(pt1[0], pt1[1], pt1[0], pt2[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt1[0], pt2[1], pt2[0], pt2[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt2[1], pt2[0], pt1[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt1[1], pt1[0], pt1[1], 1, true,COLOR_TABLE[i%30]);
            }
        }
        if(draw_merged_free_leaf) {
            auto merged_free_leaf_nodes = sbt->merged_block_ptrs_;
            for(int i=0; i<merged_free_leaf_nodes.size(); i++) {
                const auto& block_ptr = merged_free_leaf_nodes[i];
                const Pointi<2> pt1 = block_ptr->min_pt_, pt2 = block_ptr->max_pt_;// + Pointi<2>{1, 1};
                canvas.drawGridLine(pt1[0], pt1[1], pt1[0], pt2[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt1[0], pt2[1], pt2[0], pt2[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt2[1], pt2[0], pt1[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt1[1], pt1[0], pt1[1], 1, true,COLOR_TABLE[i%30]);
            }
//            for(int x=0; x<dim[0]; x++) {
//                for(int y=0; y<dim[1]; y++) {
//                    Pointi<2> pt{x, y};
//                    auto block_ptr = sbt->getInternalBlockPtr(pt);
//                    if(block_ptr == nullptr) { continue; }
//                    if(block_ptr->merged_block_id_ != -1) {
//                        Id indicator = block_ptr->merged_block_id_;
//                        canvas.drawGrid(pt[0], pt[1], COLOR_TABLE[indicator%30]);
//                    }
//                }
//            }
        }
        if(triger_varify) {
            triger_varify = false;
//            Id total_index = getTotalIndexOfSpace<2>(dim);
//            std::cout << "total_index = " << total_index << std::endl;
//            std::vector<bool> temp_map(total_index, false);
//            std::cout << "temp_map.size() = " << temp_map.size() << std::endl;
//            Pointis<2> occ_pts = dynamic_obstacles.getCurrentOccupationPoints();
//            for(const auto& occ_pt : occ_pts) {
//                temp_map[PointiToId(occ_pt, dim)] = true;
//            }
//            auto is_occupied_temp = [=](const Pointi<2> & pt) -> bool {
//                if(isOutOfBoundary<2>(pt, dim)) {
//                    return true;
//                }
//                Id temp_id = PointiToId(pt, dim);
//                //std::cout << "temp_map.size() = " << temp_map.size() << ", temp_id = " << temp_id << std::endl;
//                return temp_map[temp_id];
//            };

            std::thread t(varify_thread<2, SBT>, dim, sbt, dynamic_obstacles, is_occupied); // start varify thread
            t.detach();
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
                mst.reset();
                for(const auto& pre_pt : dynamic_obstacles.getNewPassablePoints()) {
                    if(is_occupied(pre_pt)) { continue; }
                    sbt->setOccupiedState(pre_pt, false);
                }
                //std::cout << " new pass pt = ";
                //for(const auto& pre_pt : dynamic_obstacles.getNewPassablePoints()) {
                    //if(is_occupied(pre_pt)) { continue; }
                    //std::cout << pre_pt << " ";
                //}
                //std::cout << std::endl;
                for(const auto& cur_pt : dynamic_obstacles.getNewOccupiedPoints()) {
                    sbt->setOccupiedState(cur_pt, true);
                }
                //std::cout << " new occ pt = ";
                //for(const auto& cur_pt : dynamic_obstacles.getNewOccupiedPoints()) {
                //    std::cout << cur_pt << " ";
                //}
                sbt->globalRecursiveUpdate();
                //std::cout << std::endl;
                std::cout << "finish update dynamic obstacle in " << mst.elapsed() << "ms" << std::endl;
                break;
            case 'f':
                draw_free_leaf = !draw_free_leaf;
                break;
            case 'b':
                draw_block = !draw_block;
                break;
            case 'v':
                triger_varify = true;
                break;
            case 'm':
                draw_merged_free_leaf = !draw_merged_free_leaf;
                break;
            default:
                break;
        }
    }
    delete dim;
}

// statistic about time cost of initialization of SBT / dynamic update of SBT / raw LOS / SBT's LOS

//int main() {
//    //dynamic_obstacles_2D<SpaceBinaryTreeAnyDimensionRaw<2>, TreeNode<2>>();
//    dynamic_obstacles_2D<SpaceBinaryTreeAnyDimensionRaw<2>, TreeNode<2>>();
//    return 0;
//}

std::string file_path = "../test/SBT_LOS.txt";

//int main() {
TEST(massiveSBTLOSCompareTest, test) {
    for(int i=0; i<10; i++) {

//        massiveSBTLOSCompareTest2D(10, 100, {200, 300, 400}, {10, 20, 40});
//        massiveSBTLOSCompareTest<2, SpaceBinaryTree2D>(10, 100,
//                                                       {700, 800, 1000},
//                                                       {40, 60}, 1e4, 1e3, 10);

//        massiveSBTLOSCompareTest2D(1, 1, {600}, {40});

        //massiveSBTLOSCompareTest<3>(10, 10, {50}, {10});

//        massiveSBTLOSCompareTest<2, SpaceBinaryTree2D>(10,
//                                                       1,
//                                                       {200, 300, 400, 500, 700, 800, 900, 1000},
//                                                       {10,20,30,40,50,60,70,80},
//                                                       file_path,
//                                                       1e5,
//                                                       1e3,
//                                                       1,
//                                                       true,
//                                                       4);

//        massiveSBTLOSCompareTest<3, SpaceBinaryTree3D>(5,
//                                                       1,
//                                                       {200, 300, 400, 500, 600, 700, 800},
//                                                       {10, 20, 40, 60, 80},
//                                                       file_path,
//                                                       1e5,
//                                                       1e3,
//                                                       {1,2,4,6,8,10,20,40},
//                                                       true,
//                                                       4);

        massiveSBTLOSCompareTest<2>(1,
                                    1,
                                    {300}, // 200,300,400,500,700,800,900, 1000
                                    {10, 20},
                                    file_path,
                                    1e5,
                                    1e3,
                                    {16,0},
                                    true,
                                    4);
//
//        massiveSBTLOSCompareTest<3>(1,
//                                                       1,
//                                                       {50, 100},
//                                                       {10, 20},
//                                                       file_path,
//                                                       1e5,
//                                                       1e3,
//                                                       {1,4,16,0},
//                                                       true,
//                                                       4);

    }
}

int main() {
//TEST(SBTShrink, test) {

#if 0
    DimensionLength *dim = new DimensionLength[2];

    dim[0] = 65, dim[1] = 31;

    auto is_occupied = [&](const Pointi<2> &pt) -> bool {
        if (isOutOfBoundary<2>(pt, dim)) {
            return true;
        }
        return false;
    };
#else
    auto dim = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };
#endif

    ObstaclePtrs<2> obs = {
            std::make_shared<CircleObstacle<2> >(1),
            std::make_shared<CircleObstacle<2> >(10),
            std::make_shared<CircleObstacle<2> >(10),
            std::make_shared<BlockObstacle<2> >(Pointi<2>{3, 5}),
    };

//    CircleObstaclePtrs<2> co = generateRandomCircleObstacles<2>(2, 26, 50);
//    obs.insert(obs.end(), co.begin(), co.end());
//
//    BlockObstaclePtrs<2> bo = generateRandomBlockObstacles<2>(2, Pointi<2>{2, 2}, Pointi<2>{5, 5});
//    obs.insert(obs.end(), co.begin(), co.end());

    DynamicObstacles<2> dynamic_obstacles(dim, obs);

    //dynamic_obstacles.random();
    MSTimer mst;
    int min_block_width = 2;
    std::shared_ptr<SpaceBinaryTreeShrink<2>> sbt =
            std::make_shared<SpaceBinaryTreeShrink<2>>(is_occupied, dim, min_block_width);

//    sbt->initialize();

    std::cout << "finish initialize of map in " << mst.elapsed() << "ms" << std::endl;


    float zoom_ratio = std::min(1800. / dim[0], 1000. / dim[1]);
    std::cout << "std::min(1800./dimension[0], 1000./dimension[1]) = " << zoom_ratio << std::endl;

    Canvas canvas("dynamic_obstacles_2D", dim[0], dim[1], .05, zoom_ratio);
    bool draw_pre_occupy = false,
            draw_current_occupy = true,
            draw_free_leaf = true,
            draw_block = false,
            triger_varify = false,
            draw_merged_free_leaf = true;

    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
        if(draw_pre_occupy) {
            //canvas.drawGrids(dynamic_obstacles.getPreviousOccupationPoints());
            Id total_index = getTotalIndexOfSpace<2>(dim);
            for(const auto& pre_id : dynamic_obstacles.pre_occ_ids_) {
                Pointi<2> pt = IdToPointi<2>(pre_id, dim);
                int x = pt[0], y = pt[1];
                canvas.drawGrid(x, y, cv::Vec3b::all(0));
            }
            canvas.drawGrids(dynamic_obstacles.previous_center_pts_, COLOR_TABLE[1]);
        }
        if(draw_current_occupy) {
//            canvas.drawGrids(dynamic_obstacles.getCurrentOccupationPoints());
            Id total_index = getTotalIndexOfSpace<2>(dim);
            for(int id=0; id<total_index; id++) {
                if(dynamic_obstacles.current_map_[id] == false) { continue; }
                Pointi<2> pt = IdToPointi<2>(id, dim);
                int x = pt[0], y = pt[1];
                canvas.drawGrid(x, y, cv::Vec3b::all(0));
            }
            canvas.drawGrids(dynamic_obstacles.current_center_pts_, COLOR_TABLE[0]);
        }
        if(draw_free_leaf) {
            std::vector<TreeNodePtr<2> > free_leaf_nodes = sbt->sbt_ptr_->getAllPassableLeafNodes();
            BlockPtrs<2> block_ptrs;
            for(const auto& leaf_node : free_leaf_nodes) {
                BlockPtr<2> block_ptr = std::make_shared<Block<2> >();
                block_ptr->min_ = leaf_node->base_pt_ * sbt->pow_2_[sbt->min_block_depth_width_];

                int width_of_space = sbt->pow_2_[sbt->min_block_depth_width_ + sbt->sbt_ptr_->max_depth_-leaf_node->depth_];
                Pointi<2> offset; offset.setAll(width_of_space-1);

                block_ptr->max_ = block_ptr->min_ + offset;

                //assert((!is_occupied(block_ptr->min_)) && (!is_occupied(block_ptr->max_)));
                block_ptrs.push_back(block_ptr);
            }
            for(int i=0; i<block_ptrs.size(); i++) {
                const auto& block_ptr = block_ptrs[i];
                const Pointi<2> pt1 = block_ptr->min_, pt2 = block_ptr->max_;// + Pointi<2>{1, 1};
                canvas.drawGridLine(pt1[0], pt1[1], pt1[0], pt2[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt1[0], pt2[1], pt2[0], pt2[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt2[1], pt2[0], pt1[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt1[1], pt1[0], pt1[1], 1, true,COLOR_TABLE[i%30]);
            }
        }
        if(draw_merged_free_leaf) {
            auto merged_free_leaf_nodes = sbt->sbt_ptr_->merged_block_ptrs_;
            for(int i=0; i<merged_free_leaf_nodes.size(); i++) {
                const auto& block_ptr = merged_free_leaf_nodes[i];
//                const Pointi<2> pt1 = block_ptr->min_pt_*sbt->sbt_ptr_->pow_2_[min_block_width],
//                                pt2 = block_ptr->max_pt_*sbt->sbt_ptr_->pow_2_[min_block_width];// + Pointi<2>{1, 1};
                const Pointi<2> pt1 = block_ptr->min_pt_, pt2 = block_ptr->max_pt_;// + Pointi<2>{1, 1};
                canvas.drawGridLine(pt1[0], pt1[1], pt1[0], pt2[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt1[0], pt2[1], pt2[0], pt2[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt2[1], pt2[0], pt1[1], 1, true,COLOR_TABLE[i%30]);
                canvas.drawGridLine(pt2[0], pt1[1], pt1[0], pt1[1], 1, true,COLOR_TABLE[i%30]);
            }
//            for(int x=0; x<dim[0]; x++) {
//                for(int y=0; y<dim[1]; y++) {
//                    Pointi<2> pt{x, y};
//                    auto merged_block_ptr = sbt->getInternalMergedBlockPtr(pt);
//                    if(merged_block_ptr == nullptr) { continue; }
//                    Pointi<2> min = merged_block_ptr->min_pt_, max = merged_block_ptr->max_pt_;
//                    int indicator = sbt->sbt_ptr_->getInternalBlockPtr(min)->merged_block_id_;
//                    canvas.drawGrid(pt[0], pt[1], COLOR_TABLE[indicator%30]);
//                }
//            }
        }
        char key = canvas.show(30);
        switch (key) {
            case 'p':
                draw_pre_occupy = !draw_pre_occupy;
                break;
            case 'c':
                draw_current_occupy = !draw_current_occupy;
                break;
            case 'f':
                draw_free_leaf = !draw_free_leaf;
                break;
            case 'b':
                draw_block = !draw_block;
                break;
            case 'v':
                triger_varify = true;
                break;
            case 32: // 32 means space
                dynamic_obstacles.random();
                mst.reset();
                for(const auto& pre_pt : dynamic_obstacles.getNewPassablePoints()) {
                    if(is_occupied(pre_pt)) { continue; }
                    sbt->setOccupiedState(pre_pt, false);
                }
                std::cout << " new pass pt = ";
                for(const auto& pre_pt : dynamic_obstacles.getNewPassablePoints()) {
                if(is_occupied(pre_pt)) { continue; }
                std::cout << pre_pt << " ";
                }
                std::cout << std::endl;
                for(const auto& cur_pt : dynamic_obstacles.getNewOccupiedPoints()) {
                    sbt->setOccupiedState(cur_pt, true);
                }
                std::cout << " new occ pt = ";
                for(const auto& cur_pt : dynamic_obstacles.getNewOccupiedPoints()) {
                    std::cout << cur_pt << " ";
                }
                std::cout << std::endl;
                sbt->globalRecursiveUpdate();
                std::cout << "finish update dynamic obstacle in " << mst.elapsed() << "ms" << std::endl;
                break;
            case 'm':
                draw_merged_free_leaf = !draw_merged_free_leaf;
                break;
            default:
                break;
        }
    }
}