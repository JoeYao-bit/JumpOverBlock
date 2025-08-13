//
// Created by yaozhuo on 2025/4/29.
//

#include "gtest/gtest.h"
#include "../algorithm/space_binary_tree/space_binary_tree_raw.h"
#include "../algorithm/space_binary_tree/space_binary_tree.h"

#include "dynamic_obstacles.h"
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


#if 0
PictureLoader loader(map_test_config.at("map_path"), is_grid_occupied2);
#else
TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
#endif



// statistic about time cost of initialization of SBT / dynamic update of SBT / raw LOS / SBT's LOS

//int main() {
//    //dynamic_obstacles_2D<SpaceBinaryTreeAnyDimensionRaw<2>, TreeNode<2>>();
//    dynamic_obstacles_2D<SpaceBinaryTreeAnyDimension<2>, TreeNodeNew<2>>();
//    return 0;
//}

std::string file_path = "../test/SBT_LOS";

// 1. 在Ubuntu下使用Valgrind的Massif工具分析CMake编译的C++程序内存占用，需按以下步骤操作：
// set(CMAKE_BUILD_TYPE Debug)  # 在CMakeLists.txt中设置
//# 或通过命令行指定
//cmake -DCMAKE_BUILD_TYPE=Debug ..


// 2. 使用Valgrind Massif运行程序
// valgrind --tool=massif --massif-out-file=massif.out ./your_program [参数]


// 3. 图形界面可直观展示内存分配趋势和热点函数。
// massif-visualizer massif.out
int main() {
//TEST(massiveSBTLOSCompareTest, test) {
    for(int i=0; i<100; i++) {

//        massiveSBTLOSCompareTest2D(10, 100, {200, 300, 400}, {10, 20, 40});
//        massiveSBTLOSCompareTest<2, SpaceBinaryTree2D>(10, 100,
//                                                       {700, 800, 1000},
//                                                       {40, 60}, 1e4, 1e3, 10);

//        massiveSBTLOSCompareTest2D(1, 1, {600}, {40});

        //massiveSBTLOSCompareTest<3>(10, 10, {50}, {10});
 
    //   massiveSBTLOSCompareTest<2>(1,
    //                                                  1,
    //                                                  {200, 400, 600, 800, 1000},
    //                                                  {10,20,30,40,50,60,70,80},
    //                                                  file_path,
    //                                                  1e5,
    //                                                  1e3,
    //                                                  {1, 2, 4, 16},
    //                                                  true,
    //                                                  4);

//       massiveSBTLOSCompareTest<3>(5,
//                                                      1,
//                                                      {400, 600, 800},
//                                                      {20, 40},
//                                                      file_path,
//                                                      1e5,
//                                                      1e3,
//                                                      {16},
//                                                      true,
//                                                      4);

        // massiveSBTLOSCompareTest<2>(5,
        //                               1,
        //                               {200,400,600,800,1000}, // 200,400,600,800,1000
        //                               {1, 5, 10, 20, 40, 80, 160}, // 1, 5, 10, 20, 40, 80, 160
        //                               file_path,
        //                               1e4,
        //                               1e3,
        //                               {1, 4, 16, 64, 0}, // {1, 4, 16, 64, 0}
        //                               true,
        //                               4);

        // massiveSBTLOSCompareTest<3>(1,
        //                                              1,
        //                                              {400, 600, 800, 1000}, // {200, 400, 600, 800, 1000}
        //                                              {5, 40, 80, 160, 320},
        //                                              file_path,
        //                                              1e5,
        //                                              1e3,
        //                                              {1,4,16,64,0},
        //                                              true,
        //                                              4);

        int width_of_space = 1000;
        massiveSBTLOSCompareTest<2>(1,
                                                     10,
                                                     {width_of_space},
                                                     {1, 5, 10, 20, 40, 80, 160},
                                                     file_path,
                                                     1e5,
                                                     1e3,
                                                     {2},
                                                     true,
                                                     4);

        // massiveSBTLOSCompareTest<3>(1,
        //                                              1,
        //                                              {width_of_space}, // {200, 400, 600, 800, 1000}
        //                                              {10, 40, 160, 320, 640, 960, 1280},
        //                                              file_path,
        //                                              1e5,
        //                                              1e3,
        //                                              {2},
        //                                              true,
        //                                              4);

    }
}