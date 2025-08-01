//
// Created by yaozhuo on 8/1/25.
//

#include "../test/test_data.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/visualization/canvas/canvas.h"

#include "../algorithm/space_binary_tree/space_binary_tree_raw.h"
#include "../algorithm/space_binary_tree/space_binary_tree.h"


#include "../third_party/LazyThetaStar/pathfinding.hpp"
#include "../third_party/LazyThetaStar/JOB_adapter.h"


using namespace freeNav::JOB;
using namespace freeNav;



// MapTestConfig_ost003d // ok
// MapTestConfig_orz900d // ok
// MapTestConfig_lak303d // ok
// MapTestConfig_den520d // ok
// MapTestConfig_den312d // ok
// MapTestConfig_Shanghai_0_512 // ok


auto map_test_config = MapTestConfig_Shanghai_0_512;
std::string vis_file_path    = map_test_config.at("vis_path");

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};


TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
int zoom_rate = 1;

bool set_pt1 = true;
bool new_pair = false;
bool plan_finish = false;
bool draw_path = true;

//Pointi<2> pt1 = {0, 0}, pt2 = {0, 0};
Pointi<2> pt1 = {127, 272}, pt2 = {197, 397};
//Pointi<2> pt1 = {13, 3}, pt2 = {20, 12};

int main() {

    auto dimension = loader.getDimensionInfo();

    auto is_occupied = [](const Pointi<2> & pt) -> bool {
        return loader.isOccupied(pt);
    };

    MSTimer mst;
    SpaceBinaryTree2DRaw sbt_raw(is_occupied, dimension, 0);
    sbt_raw.initialize();

    std::cout << "raw SBT init in " << mst.elapsed() << "ms" << std::endl;

    mst.reset();
    SpaceBinaryTree2D sbt(is_occupied, dimension, 4);
    sbt.initialize();
    std::cout << "new SBT init in " << mst.elapsed() << "ms" << std::endl;
    int count_block = 0;

    auto is_line_free_raw = [&](const Pointi<2> & p1, const Pointi<2> & p2) -> bool {
        return !sbt_raw.lineCrossObstacleRaw(p1, p2, is_occupied);
    };

    auto is_line_free_raw_sbt = [&](const Pointi<2> & p1, const Pointi<2> & p2) -> bool {
        return !sbt_raw.lineCrossObstacleSBT(p1, p2, is_occupied, count_block);
    };

    auto is_line_free_new_sbt = [&](const Pointi<2> & p1, const Pointi<2> & p2) -> bool {
        return !sbt.lineCrossObstacleSBT(p1, p2, is_occupied, count_block);
    };

    LazyThetaStar::MyAdaptor2D my_adapter2D(dimension, is_occupied, is_line_free_new_sbt);
    LazyThetaStar::Pathfinder pathfinder(my_adapter2D, 100.f /*weight*/);

    Canvas canvas("2D planner test",dimension[0], dimension[1], .05, zoom_rate);

    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = x;
                pt1[1] = y;
                set_pt1 = false;
                plan_finish = false;
                std::cout << "get point " << x << ", " << y << std::endl;
            } else {
                pt2[0] = x;
                pt2[1] = y;
                set_pt1 = true;
                std::cout << "get point " << x << ", " << y << std::endl;
                new_pair = true;
                plan_finish = false;
            }
        }
    };

    canvas.setMouseCallBack(callback);

    bool plan_path = false;
    Pointis<2> result_path;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied);

        //canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, 2, cv::Vec3b(0,255,0));
        canvas.drawCircleInt(pt1[0], pt1[1], 5, true, -1, cv::Vec3b(0,255,0));
        canvas.drawCircleInt(pt2[0], pt2[1], 5, true, -1, cv::Vec3b(255,0, 0));

        if(plan_path) {
            plan_path = false;
            result_path.clear();
            auto nodePath = pathfinder.search(PointiToId<2>(pt1, dimension), PointiToId<2>(pt2, dimension));
            if(nodePath.empty()) {
                std::cout << "plan from " << pt1 << " to " << pt2 << " failed" << std::endl;
            } else {
                std::cout << "plan from " << pt1 << " to " << pt2 << " success" << std::endl;
                for(const auto& id : nodePath) {
                    result_path.push_back(IdToPointi<2>(id, dimension));
                }
            }
        }
        if(draw_path) {
            if(result_path.size() >= 2) {
                for(int i=0; i<result_path.size()-1; i++) {
                    int x1 = result_path[i][0];
                    int y1 = result_path[i][1];
                    int x2 = result_path[i+1][0];
                    int y2 = result_path[i+1][1];

                    canvas.drawLineInt(x1, y1, x2, y2, true, 2, cv::Vec3b(0,255,0));
                }
            }
        }
        char key = canvas.show(30);
        switch (key) {
            case 32: // space
                plan_path = !plan_path;
                break;
            case 'p':
                draw_path = !draw_path;
            default:
                break;
        }
    }

}