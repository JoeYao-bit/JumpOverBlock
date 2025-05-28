//
// Created by yaozhuo on 2025/4/29.
//

#include "gtest/gtest.h"
#include "../algorithm/space_binary_tree/space_binary_tree.h"
#include "dynamic_obstacles.h"
#include "dependencies.h"

#include <thread>

// dynamic map
// time cost of update dynamic map
// time cost of raw LOS check and SpaceBinaryTree's LOS check

using namespace freeNav::JOB;
using namespace freeNav;

template<Dimension N>
void varify_thread(DimensionLength* temp_dim,
                   const IS_OCCUPIED_FUNC<N>& isoc_temp,
                   const SpaceBinaryTreePtr<N>& sbt) {
//    SpaceBinaryTreeVarify(temp_dim, isoc_temp, sbt);
    LOSCompare<N>(temp_dim, isoc_temp, sbt);

}

// statistic about time cost of initialization of SBT / dynamic update of SBT / raw LOS / SBT's LOS


std::string file_path = "../test/SBT_LOS.txt";

int main() {
//TEST(massiveSBTLOSCompareTest, test) {
    for(int i=0; i<1; i++) {

        //  massiveSBTLOSCompareTest<2, SpaceBinaryTree2D>(5,
        //                                                 1,
        //                                                 {200, 400, 600, 800, 1000},
        //                                                 {10, 20, 40, 60, 80, 160},
        //                                                 file_path,
        //                                                 1e5,
        //                                                 1e3,
        //                                                 {1,2,3,4},
        //                                                 true,
        //                                                 4);


       for(int k=0; k<5; k++) {

           massiveSBTLOSCompareTest<3, SpaceBinaryTree3D>(1,
                                                          1,
                                                          {800},
                   //{600, 700, 800},
                                                          {40, 80, 160},
                                                          file_path,
                                                          1e5,
                                                          1e3,
                                                          {1, 2, 3, 4},
                                                          true,
                                                          4);
       }


    }
}