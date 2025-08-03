//
// Created by yaozhuo on 2023/7/24.
//

#include "../freeNav-base/dependencies/massive_test_interfaces.h"
#include "../algorithm/surface_process_jump_block.h"
#include "../test/test_data.h"

using namespace freeNav;
using namespace freeNav::JOB;

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


// any dimension LOS check compare, only raw LOS check and block check
template <Dimension N>
bool SingleMapLOSCheckAnyDimensionRandomMap(DimensionLength* dimension,
                                            const IS_OCCUPIED_FUNC<N>& is_occupied_func,
                                            const SET_OCCUPIED_FUNC<N>& set_occupied_func,
                                            const Pointis<N>& occ_voxels,
                                            const IdSet& occ_voxel_ids,
                                            const std::string& block_file_path,
                                            const std::string& out_put_path,
                                            PathLen min_block_width = 3,
                                            int shrink_level = 3,
                                            int repeat = 100, int test_cases = 10000, int random_select = 100) {
    struct timezone tz;
    struct timeval tv_pre;
    struct timeval tv_after;

    auto surface_processor =
            std::make_shared<SurfaceProcessorSparseWithJumpBlockGreedyShrink<N> >(dimension,
                    is_occupied_func,
                    set_occupied_func,
                    occ_voxels,
                    occ_voxel_ids,
                    shrink_level,//atoi(config.at("shrink_level").c_str()),
                    min_block_width,//atof(config.at("minimum_block_width").c_str()),
                    block_file_path,
                    true);

    auto los_with_jump_block = [&](const Pointi<N> &start,
                                   const Pointi<N> &target,
                                   Pointis<N> &path,
                                   Statistic &statistic,
                                   OutputStream &output_stream) {
        path.clear();
        statistic.clear();
        output_stream.clear();
        path.push_back(start);
        path.push_back(target);
        bool is_collide;
        Pointis<N> visited_pt;
        int count_of_block;
        gettimeofday(&tv_pre, &tz);
        for(int i=0; i<repeat; i++) {
            is_collide = surface_processor->lineCrossObstacleWithVisitedPoint(start, target, visited_pt, count_of_block);
        }
        gettimeofday(&tv_after, &tz);
        double los_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        statistic.push_back(los_cost);
        std::stringstream ss;
        ss << "JumpBlock ";
        for(int dim=0; dim<N; dim++) {
            ss << start[dim]  << " ";
        }
        for(int dim=0; dim<N; dim++) {
            ss << target[dim]  << " ";
        }
        ss << los_cost << " " << visited_pt.size() << " " << count_of_block << " " << (is_collide ? 1 : 0) << " " ;
        output_stream = ss.str();
    };

    auto los_raw = [&](const Pointi<N> &start,
                       const Pointi<N> &target,
                       Pointis<N> &path,
                       Statistic &statistic,
                       OutputStream &output_stream) {
        path.clear();
        statistic.clear();
        output_stream.clear();
        path.push_back(start);
        path.push_back(target);
        bool is_collide;
        int count_of_block = 0;
        Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();
        gettimeofday(&tv_pre, &tz);
        for(int i=0; i<repeat; i++) {
            is_collide = LineCrossObstacle(start, target, is_occupied_func, neighbor);
        }
        gettimeofday(&tv_after, &tz);
        double los_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        statistic.push_back(los_cost);
        std::stringstream ss;
        ss << "Raw ";
        for(int dim=0; dim<N; dim++) {
            ss << start[dim]  << " ";
        }
        for(int dim=0; dim<N; dim++) {
            ss << target[dim]  << " ";
        }
        Line<N> line(start, target);
        ss << los_cost << " " << line.step << " " << count_of_block << " " << (is_collide ? 1 : 0) << " ";
        output_stream = ss.str();
    };

    Point2PointPathPlannings<N, Pointi<N>, Pointi<N> > path_plannings = {los_with_jump_block,
                                                                         los_raw
    };

    StatisticSS statisticss;
    OutputStreamSS output_streamss;
    SceneTest_Random<N>(dimension, is_occupied_func, test_cases, path_plannings, statisticss, output_streamss, random_select);

    std::ofstream os(out_put_path);
    //os << "TYPE START_X START_Y TARGET_X TARGET_Y PATH_LENGTH RESET_TIME INITIAL_TIME SEARCH_TIME" << std::endl;
    for (const auto &multi_method_output : output_streamss) {
        for (const auto method_output : multi_method_output) {
            os << method_output << std::endl;
        }
    }
    os.close();
    return true;
}

template <Dimension N>
bool SingleMapLOSCheckRandomMap(DimensionLength* dimension,
                                Id cubic_half_width,
                                Id cubic_number,
                                const std::string& random_file_path,
                                const std::string& block_file_path,
                                const std::string& test_data_path,
                                PathLen min_block_width = 3,
                                int shrink_level = 3,
                                int repeat = 100, int test_cases = 10000, int random_select = 100) {
    RandomMapGenerator<N> random_map(dimension, cubic_half_width, cubic_number, random_file_path, false);
    Id total_index = getTotalIndexOfSpace<N>(dimension);
    std::cout << "cubic number: " << cubic_number << ", occ ratio : " << 100.*random_map.occ_grids_.size()/total_index << "%" << std::endl;
    auto is_occupied_random = [&](const Pointi<N> & pt) -> bool {
        if(isOutOfBoundary(pt, dimension)) { return true; }
        Id id = PointiToId(pt, dimension);
        return random_map.occ_ids_.find(id) != random_map.occ_ids_.end();
    };

    IS_OCCUPIED_FUNC<N> is_occupied_func = is_occupied_random;

    SET_OCCUPIED_FUNC<N> set_occupied_func; // useless function, no need to set

    return SingleMapLOSCheckAnyDimensionRandomMap(dimension, is_occupied_func, set_occupied_func,
                                                  random_map.occ_grids_, random_map.occ_ids_,
                                                  block_file_path, test_data_path,
                                                  min_block_width, shrink_level,
                                                  repeat, test_cases, random_select);

}

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