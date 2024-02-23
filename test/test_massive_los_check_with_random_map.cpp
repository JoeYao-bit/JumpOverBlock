//
// Created by yaozhuo on 2023/7/24.
//

#include "dependencies/massive_test_interfaces.h"
#include "dependencies/test_data.h"

using namespace freeNav;
using namespace freeNav::RimJump;

/*
 *     bool SingleMapLOSCheck3DRandomMap(DimensionLength* dimension,
                                      Id cubic_half_width,
                                      Id cubic_number,
                                      const std::string& random_file_path,
                                      const std::string& block_file_path,
                                      const std::string& test_data_path,
                                      PathLen min_block_width = 3,
                                      int shrink_level = 3,
                                      int repeat = 100, int test_cases = 10000, int random_select = 100)
 * */

RandomMapTestConfigs<2> random_maps_2d = {
                                       grid_2D_1,
                                       grid_2D_2,
                                       grid_2D_3,
                                       grid_2D_4,
                                       grid_2D_5,
                                       grid_2D_6,
                                       grid_2D_7,
                                         grid_2D_8,
//                                         grid_2D_8_1,
//                                         grid_2D_8_2,
//                                         grid_2D_8_3,
//                                         grid_2D_8_4,
//                                         grid_2D_8_5,
//                                         grid_2D_8_6
                                         };

RandomMapTestConfigs<3> random_maps_3d = {
                                          grid_3D_1,
                                          grid_3D_2,
                                          grid_3D_3,
                                          grid_3D_4,
                                          grid_3D_5,
                                          grid_3D_6,
                                          grid_3D_7,
                                          grid_3D_8,
//                                          grid_3D_8_1,
//                                          grid_3D_8_2,
//                                          grid_3D_8_3,
//                                          grid_3D_8_4
                                          };

RandomMapTestConfigs<4> random_maps_4d = {
                                          grid_4D_1,
                                          grid_4D_2,
                                          grid_4D_3,
                                          grid_4D_4,
                                          grid_4D_5,
                                          grid_4D_6,
                                          grid_4D_7,
                                          grid_4D_8
};

int main() {
    std::vector<std::vector<double> > ratioss;

    auto random_maps_2 = random_maps_2d;//random_maps_4d;
    ratioss.clear();
    for(auto& config : random_maps_2) {
        std::cout << config.name_ << ":" << std::endl;
        SingleMapLOSCheckRandomMap<2>(config.dim_,
                                      config.cubic_half_width_,
                                      config.cubic_number_,
                                      config.random_file_path_,
                                      config.block_file_path_,
                                      config.test_data_path_,
                                      config.min_block_width_,
                                      config.shrink_level_,
                                      50,
                                      50000);
        const auto& ratios = SingleMapLosTestDataAnalysis<2>(config.test_data_path_);
        ratioss.push_back(ratios);
        if (!ratios.empty()) {
            for (const auto &ratio : ratios) {
                std::cout << ratio << " ";
            }
            std::cout << std::endl;
        }
    }

//    auto& random_maps_3 = random_maps_3d;//random_maps_2d;//random_maps_4d;
//    ratioss.clear();
//    for(auto& config : random_maps_3) {
//        std::cout << config.name_ << ":" << std::endl;
//        SingleMapLOSCheckRandomMap<3>(config.dim_,
//                                      config.cubic_half_width_,
//                                      config.cubic_number_,
//                                      config.random_file_path_,
//                                      config.block_file_path_,
//                                      config.test_data_path_,
//                                      config.min_block_width_,
//                                      config.shrink_level_,
//                                      50,
//                                      50000);
//        const auto& ratios = SingleMapLosTestDataAnalysis<3>(config.test_data_path_);
//        ratioss.push_back(ratios);
//        if (!ratios.empty()) {
//            for (const auto &ratio : ratios) {
//                std::cout << ratio << " ";
//            }
//            std::cout << std::endl;
//        }
//    }

//    auto& random_maps_4 = random_maps_4d;//random_maps_2d;//random_maps_4d;
//    ratioss.clear();
//    for(auto& config : random_maps_4) {
//        std::cout << config.name_ << ":" << std::endl;
//        SingleMapLOSCheckRandomMap<4>(config.dim_,
//                                      config.cubic_half_width_,
//                                      config.cubic_number_,
//                                      config.random_file_path_,
//                                      config.block_file_path_,
//                                      config.test_data_path_,
//                                      config.min_block_width_,
//                                      config.shrink_level_,
//                                      50,
//                                      50000);
//        const auto& ratios = SingleMapLosTestDataAnalysis<4>(config.test_data_path_);
//        ratioss.push_back(ratios);
//        if (!ratios.empty()) {
//            for (const auto &ratio : ratios) {
//                std::cout << ratio << " ";
//            }
//            std::cout << std::endl;
//        }
//    }

}