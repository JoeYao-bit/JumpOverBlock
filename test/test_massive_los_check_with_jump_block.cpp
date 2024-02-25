//
// Created by yaozhuo on 2023/5/7.
//


#include "../test/test_data.h"
#include "../freeNav-base/dependencies/massive_test_interfaces.h"
#include "../freeNav-base/dependencies/random_map_generator.h"
#include "../freeNav-base/dependencies/3d_textmap/voxel_loader.h"

#include "../algorithm/line_of_sight_jump_between_block.h"
#include "../algorithm/surface_process_jump_block.h"

using namespace freeNav::JOB;
using namespace freeNav;

auto is_char_occupied = [](const char& value) -> bool {
    if (value != '.' && value != 'G' && value != 'S') return true;
    return false;
};

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

bool SingleMapLOSCheck3D(DimensionLength* dimension,
                         const IS_OCCUPIED_FUNC<3>& is_occupied_func,
                         const SET_OCCUPIED_FUNC<3>& set_occupied_func,
                         const Pointis<3>& occ_voxels,
                         const IdSet& occ_voxel_ids,
                         const std::string& block_file_path,
                         const std::string& block_file_path_oc,
                         const std::string& out_put_path,
                         PathLen min_block_width = 3,
                         int shrink_level = 3,
                         int repeat = 100, int test_cases = 10000, int random_select = 100) {

    auto surface_processor =
            std::make_shared<SurfaceProcessorSparseWithJumpBlockGreedyShrink<3> >(dimension,
                                                                                  is_occupied_func,
                                                                                  set_occupied_func,
                                                                                  occ_voxels,
                                                                                  occ_voxel_ids,
                                                                                  shrink_level,//atoi(config.at("shrink_level").c_str()),
                                                                                  min_block_width,//atof(config.at("minimum_block_width").c_str()),
                                                                                  block_file_path,
                                                                                  true);
//    auto surface_processor =
//            std::make_shared<SurfaceProcessorSparseWithJumpBlockGreedy<3> >(dimension,
//                                                                                  is_occupied_func,
//                                                                                  set_occupied_func,
//                                                                                  occ_voxels,
//                                                                                  occ_voxel_ids,
//                                                                                  min_block_width,//atof(config.at("minimum_block_width").c_str()),
//                                                                                  block_file_path,
//                                                                                  true);
    auto los_with_jump_block = [&](const Pointi<3> &start,
                                   const Pointi<3> &target,
                                   Pointis<3> &path,
                                   Statistic &statistic,
                                   OutputStream &output_stream) {
        path.clear();
        statistic.clear();
        output_stream.clear();
        path.push_back(start);
        path.push_back(target);
        bool is_collide;
        Pointis<3> visited_pt;
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
        for(int dim=0; dim<3; dim++) {
            ss << start[dim]  << " ";
        }
        for(int dim=0; dim<3; dim++) {
            ss << target[dim]  << " ";
        }
        ss << los_cost << " " << visited_pt.size() << " " << count_of_block << " " << (is_collide ? 1 : 0) << " " ;
        output_stream = ss.str();
    };

    auto surface_processor_octomap =
            std::make_shared<SurfaceProcessorSparseWithJumpBlockOctoMap>(dimension, is_occupied_func, set_occupied_func,
                                                                         occ_voxels,
                                                                         occ_voxel_ids,
                                                                         min_block_width,//atoi(map_test_config.at("minimum_block_width").c_str()),
                                                                         block_file_path_oc,//map_test_config.at("block_path").c_str(),
                                                                         true);

    auto los_with_jump_block_octomap = [&](const Pointi<3> &start,
                                           const Pointi<3> &target,
                                           Pointis<3> &path,
                                           Statistic &statistic,
                                           OutputStream &output_stream) {
        path.clear();
        statistic.clear();
        output_stream.clear();
        path.push_back(start);
        path.push_back(target);
        bool is_collide;
        Pointis<3> visited_pt;
        int count_of_block;
        gettimeofday(&tv_pre, &tz);
        for(int i=0; i<repeat; i++) {
            is_collide = surface_processor_octomap->lineCrossObstacleWithVisitedPoint(start, target, visited_pt, count_of_block);
        }
        gettimeofday(&tv_after, &tz);
        double los_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        statistic.push_back(los_cost);
        std::stringstream ss;
        ss << "JumpBlockOcM ";
        for(int dim=0; dim<3; dim++) {
            ss << start[dim]  << " ";
        }
        for(int dim=0; dim<3; dim++) {
            ss << target[dim]  << " ";
        }
        ss << los_cost << " " << visited_pt.size() << " " << count_of_block << " " << (is_collide ? 1 : 0) << " " ;
        output_stream = ss.str();
    };

    auto los_raw = [&](const Pointi<3> &start,
                       const Pointi<3> &target,
                       Pointis<3> &path,
                       Statistic &statistic,
                       OutputStream &output_stream) {
        path.clear();
        statistic.clear();
        output_stream.clear();
        path.push_back(start);
        path.push_back(target);
        bool is_collide;
        int count_of_block = 0;
        Pointis<2> neighbor = GetNeightborOffsetGrids<2>();
        gettimeofday(&tv_pre, &tz);
        for(int i=0; i<repeat; i++) {
            is_collide = LineCrossObstacle(start, target, is_occupied_func, neighbor);
        }
        gettimeofday(&tv_after, &tz);
        double los_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        statistic.push_back(los_cost);
        std::stringstream ss;
        ss << "Raw ";
        for(int dim=0; dim<3; dim++) {
            ss << start[dim]  << " ";
        }
        for(int dim=0; dim<3; dim++) {
            ss << target[dim]  << " ";
        }
        Line<3> line(start, target);
        ss << los_cost << " " << line.step << " " << count_of_block << " " << (is_collide ? 1 : 0) << " ";
        output_stream = ss.str();
    };

    Point2PointPathPlannings<3, Pointi<3>, Pointi<3> > path_plannings = {los_with_jump_block,
                                                                         los_with_jump_block_octomap,
                                                                         los_raw
                                                                         };

    StatisticSS statisticss;
    OutputStreamSS output_streamss;
    SceneTest_Random<3>(dimension, is_occupied_func, test_cases, path_plannings, statisticss, output_streamss, random_select);

    std::ofstream os(out_put_path);
    //os << "TYPE START_X START_Y TARGET_X TARGET_Y PATH_LENGTH RESET_TIME INITIAL_TIME SEARCH_TIME" << std::endl;
    for (const auto &multi_method_output : output_streamss) {
        for (const auto method_output : multi_method_output) {
            os << method_output << std::endl;
        }
    }
    os.close();
    return true;
};

bool SingleMapLOSCheck3DTextMap(const SingleMapTestConfig <3> &map_test_config, int repeat = 100, int test_cases = 10000, int random_select = 100) {
    TextMapLoader_3D tl(map_test_config.at("map_path"));
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    auto dimension = tl.getDimensionInfo();

    IS_OCCUPIED_FUNC<3> is_occupied_func;

    SET_OCCUPIED_FUNC<3> set_occupied_func;

    auto is_occupied = [&tl](const Pointi<3> &pt) -> bool { return tl.isOccupied(pt); };
    is_occupied_func = is_occupied;

    auto set_occupied = [&tl](const Pointi<3> &pt) { tl.setOccupied(pt); };
    set_occupied_func = set_occupied;

    return SingleMapLOSCheck3D(dimension, is_occupied_func, set_occupied_func,
                               tl.occ_voxels_, tl.occ_voxel_ids_,
                               map_test_config.at("block_path"),
                               map_test_config.at("block_path_oc"),
                               map_test_config.at("los_output_path"),
                               atof(map_test_config.at("minimum_block_width").c_str()),
                               atof(map_test_config.at("minimum_block_width").c_str()),//atoi(map_test_config.at("shrink_level").c_str()),
                               repeat, test_cases, random_select);

}

// los check between random uniform sample free grid
SingleMapTestConfigs<3> configs = {
            MapTestConfig_Simple,
            MapTestConfig_Complex, // AC
            MapTestConfig_A1, // AC
            MapTestConfig_A2, // AC
            MapTestConfig_A3, // AC
            MapTestConfig_A4, // AC
            MapTestConfig_A5, // AC

            MapTestConfig_BC1, //
            MapTestConfig_BC2, // AC
            MapTestConfig_DA1, // AC
            MapTestConfig_DA2, // AC
            MapTestConfig_DB1, // AC
            MapTestConfig_DB2, // AC

            MapTestConfig_DC1, // AC
            MapTestConfig_DC2, // AC
            MapTestConfig_EB1, // AC
            MapTestConfig_EB2, // AC
            MapTestConfig_EC1, // AC
            MapTestConfig_EC2, // AC
            //MapTestConfig_Full4
};



int main() {
//    for(const auto& config : configs) {
//        SingleMapLOSCheck3DTextMap(config, 100,10000);
//    }
//    for(const auto& config : configs) {
//        std::cout << config.at("map_name") << ":" << std::endl;
//        SingleMapLosTestDataAnalysis<3>(config.at("los_output_path"));
//        std::cout << std::endl;
//    }
    for(auto config : configs)
    {
        //auto config = MapTestConfig_Complex;
        std::vector<std::vector<double> > ratioss;
        //for(int i=1; i<20; i++)
        {
            std::stringstream ss;
            ss << 3;
            config.find("minimum_block_width")->second = ss.str();
            std::cout << "minimum_block_width = " << config.find("minimum_block_width")->second << std::endl;
            SingleMapLOSCheck3DTextMap(config, 100,10000);
            std::cout << config.at("map_name") << ":" << std::endl;
            const auto& ratios = SingleMapLosTestDataAnalysis<3>(config.at("los_output_path"));
            ratioss.push_back(ratios);
        }
        std::cout << config.at("map_name") << ":" << std::endl;
        for(int i=0; i<ratioss.size(); i++) {
            std::cout << " block width = " << i+1 << " ";
            const auto& ratios = ratioss[i];
            if (!ratios.empty()) {
                for (const auto &ratio : ratios) {
                    std::cout << ratio << " ";
                }
                std::cout << std::endl;
            }
        }
    }
}
