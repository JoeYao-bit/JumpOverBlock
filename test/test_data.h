//
// Created by yaozhuo on 2022/9/6.
//

#ifndef FREENAV_JOB_BASE_TEST_DATA_H
#define FREENAV_JOB_BASE_TEST_DATA_H
#include <iostream>
#include <map>
#include "../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/dependencies/massive_test_interfaces.h"

namespace freeNav {

    // Shanghai_0_512.map
    SingleMapTestConfig<2> MapTestConfig_Shanghai_0_512 =
            {
                    {"map_name",    "Shanghai_0_512"},
                    {"map_path",    "../test/test_data/Shanghai_0_512.map"},
                    {"vis_path",    "../test/test_data/Shanghai_0_512_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Shanghai_0_512.map.scen"},
                    {"output_path", "../test/test_data/Shanghai_0_512.txt"}
            };

    SingleMapTestConfig<3> MapTestConfig_Simple =

            {
                    {"map_name",    "Simple"},
                    {"map_path",    "../test/test_data/Simple.3dmap"},
                    {"vis_path",    "../test/test_data/Simple.vis"},
                    {"block_path",  "../test/test_data/Simple.block"},
                    {"block_path_oc",  "../test/test_data/Simple_oc.block"},
                    {"minimum_block_width", "28"},
                    {"los_output_path", "../test/test_data/Simple-los.txt"},
                    {"config_path", "../test/test_data/Simple.3dmap.3dscen"},
                    {"output_path", "../test/test_data/Simple.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_Complex =

            {
                    {"map_name",    "Complex"},
                    {"map_path",    "../test/test_data/Complex.3dmap"},
                    {"vis_path",    "../test/test_data/Complex.vis"},
                    {"block_path",  "../test/test_data/Complex.block"},
                    {"block_path_oc",  "../test/test_data/Complex_oc.block"},
                    {"minimum_block_width", "10"}, // optimal 26
                    {"los_output_path", "../test/test_data/Complex-los.txt"},
                    {"config_path", "../test/test_data/Complex.3dmap.3dscen"},
                    {"output_path", "../test/test_data/Complex.txt"},
                    {"shrink_level", "1"}
            };


    // A1.3dmap
    SingleMapTestConfig<3> MapTestConfig_A1 =

            {
                    {"map_name",    "A1"},
                    {"map_path",    "../test/test_data/A1.3dmap"},
                    {"vis_path",    "../test/test_data/A1.vis"},
                    {"block_path",  "../test/test_data/A1.block"},
                    {"block_path_oc",  "../test/test_data/A1_oc.block"},
                    {"minimum_block_width", "3"},
                    {"los_output_path", "../test/test_data/A1-los.txt"},{"config_path", "../test/test_data/A1.3dmap.3dscen"},
                    {"output_path", "../test/test_data/A1.txt"},
                    {"shrink_level", "3"}
            };

    // A2.3dmap
    SingleMapTestConfig<3> MapTestConfig_A2 =

            {
                    {"map_name",    "A2"},
                    {"map_path",    "../test/test_data/A2.3dmap"},
                    {"vis_path",    "../test/test_data/A2.vis"},
                    {"block_path",  "../test/test_data/A2.block"},
                    {"block_path_oc",  "../test/test_data/A2_oc.block"},
                    {"minimum_block_width", "36"},
                    {"los_output_path", "../test/test_data/A2-los.txt"},
                    {"config_path", "../test/test_data/A2.3dmap.3dscen"},
                    {"output_path", "../test/test_data/A2.txt"},
                    {"shrink_level", "1"}
            };

    // A3.3dmap
    SingleMapTestConfig<3> MapTestConfig_A3 =

            {
                    {"map_name",    "A3"},
                    {"map_path",    "../test/test_data/A3.3dmap"},
                    {"vis_path",    "../test/test_data/A3.vis"},
                    {"block_path",  "../test/test_data/A3.block"},
                    {"block_path_oc",  "../test/test_data/A3_oc.block"},
                    {"minimum_block_width", "32"}, // pre 32, 36
                    {"los_output_path", "../test/test_data/A3-los.txt"},
                    {"config_path", "../test/test_data/A3.3dmap.3dscen"},
                    {"output_path", "../test/test_data/A3.txt"},
                    {"shrink_level", "1"}
            };

    // A4.3dmap
    SingleMapTestConfig<3> MapTestConfig_A4 =

            {
                    {"map_name",    "A4"},
                    {"map_path",    "../test/test_data/A4.3dmap"},
                    {"vis_path",    "../test/test_data/A4.vis"},
                    {"block_path",  "../test/test_data/A4.block"},
                    {"block_path_oc",  "../test/test_data/A4_oc.block"},
                    {"minimum_block_width", "38"},
                    {"los_output_path", "../test/test_data/A4-los.txt"},
                    {"config_path", "../test/test_data/A4.3dmap.3dscen"},
                    {"output_path", "../test/test_data/A4.txt"},
                    {"shrink_level", "1"}
            };

    // A5.3dmap
    SingleMapTestConfig<3> MapTestConfig_A5=

            {
                    {"map_name",    "A5"},
                    {"map_path",    "../test/test_data/A5.3dmap"},
                    {"vis_path",    "../test/test_data/A5.vis"},
                    {"block_path",  "../test/test_data/A5.block"},
                    {"block_path_oc",  "../test/test_data/A5_oc.block"},
                    {"minimum_block_width", "31"},
                    {"los_output_path", "../test/test_data/A5-los.txt"},
                    {"config_path", "../test/test_data/A5.3dmap.3dscen"},
                    {"output_path", "../test/test_data/A5.txt"},
                    {"shrink_level", "1"}
            };

    // BC1.3dmap
    SingleMapTestConfig<3> MapTestConfig_BC1 =

            {
                    {"map_name",    "BC1"},
                    {"map_path",    "../test/test_data/BC1.3dmap"},
                    {"vis_path",    "../test/test_data/BC1.vis"},
                    {"block_path",  "../test/test_data/BC1.block"},
                    {"block_path_oc",  "../test/test_data/BC1_oc.block"},
                    {"minimum_block_width", "38"},
                    {"los_output_path", "../test/test_data/BC1-los.txt"},
                    {"config_path", "../test/test_data/BC1.3dmap.3dscen"},
                    {"output_path", "../test/test_data/BC1.txt"},
                    {"shrink_level", "1"}
            };

    // BC2.3dmap
    SingleMapTestConfig<3> MapTestConfig_BC2 =

            {
                    {"map_name",    "BC2"},
                    {"map_path",    "../test/test_data/BC2.3dmap"},
                    {"vis_path",    "../test/test_data/BC2.vis"},
                    {"block_path",  "../test/test_data/BC2.block"},
                    {"block_path_oc",  "../test/test_data/BC2_oc.block"},
                    {"minimum_block_width", "33"},
                    {"los_output_path", "../test/test_data/BC2-los.txt"},
                    {"config_path", "../test/test_data/BC2.3dmap.3dscen"},
                    {"output_path", "../test/test_data/BC2.txt"},
                    {"shrink_level", "1"}
            };

    // DA2.3dmap
    SingleMapTestConfig<3> MapTestConfig_DA1 =

            {
                    {"map_name",    "DA1"},
                    {"map_path",    "../test/test_data/DA1.3dmap"},
                    {"vis_path",    "../test/test_data/DA1.vis"},
                    {"block_path",  "../test/test_data/DA1.block"},
                    {"block_path_oc",  "../test/test_data/DA1_oc.block"},
                    {"minimum_block_width", "34"}, // pre 37
                    {"los_output_path", "../test/test_data/DA1-los.txt"},
                    {"config_path", "../test/test_data/DA1.3dmap.3dscen"},
                    {"output_path", "../test/test_data/DA1.txt"},
                    {"shrink_level", "1"}
            };

    // DA2.3dmap
    SingleMapTestConfig<3> MapTestConfig_DA2 =

            {
                    {"map_name",    "DA2"},
                    {"map_path",    "../test/test_data/DA2.3dmap"},
                    {"vis_path",    "../test/test_data/DA2.vis"},
                    {"block_path",  "../test/test_data/DA2.block"},
                    {"block_path_oc",  "../test/test_data/DA2_oc.block"},
                    {"minimum_block_width", "35"},
                    {"los_output_path", "../test/test_data/DA2-los.txt"},
                    {"config_path", "../test/test_data/DA2.3dmap.3dscen"},
                    {"output_path", "../test/test_data/DA2.txt"},
                    {"shrink_level", "5"}
            };

    // DB1.3dmap
    SingleMapTestConfig<3> MapTestConfig_DB1 =

            {
                    {"map_name",    "DB1"},
                    {"map_path",    "../test/test_data/DB1.3dmap"},
                    {"vis_path",    "../test/test_data/DB1.vis"},
                    {"block_path",  "../test/test_data/DB1.block"},
                    {"block_path_oc",  "../test/test_data/DB1_oc.block"},
                    {"minimum_block_width", "36"}, // pre 23
                    {"los_output_path", "../test/test_data/DB1-los.txt"},
                    {"config_path", "../test/test_data/DB1.3dmap.3dscen"},
                    {"output_path", "../test/test_data/DB1.txt"},
                    {"shrink_level", "1"}
            };

    // DB2.3dmap
    SingleMapTestConfig<3> MapTestConfig_DB2 =

            {
                    {"map_name",    "DB2"},
                    {"map_path",    "../test/test_data/DB2.3dmap"},
                    {"vis_path",    "../test/test_data/DB2.vis"},
                    {"block_path",  "../test/test_data/DB2.block"},
                    {"block_path_oc",  "../test/test_data/DB2_oc.block"},
                    {"minimum_block_width", "40"},
                    {"los_output_path", "../test/test_data/DB2-los.txt"},
                    {"config_path", "../test/test_data/DB2.3dmap.3dscen"},
                    {"output_path", "../test/test_data/DB2.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_DC1 =

            {
                    {"map_name",    "DC1"},
                    {"map_path",    "../test/test_data/DC1.3dmap"},
                    {"vis_path",    "../test/test_data/DC1.vis"},
                    {"block_path",  "../test/test_data/DC1.block"},
                    {"block_path_oc",  "../test/test_data/DC1_oc.block"},
                    {"minimum_block_width", "45"},
                    {"los_output_path", "../test/test_data/DC1-los.txt"},
                    {"config_path", "../test/test_data/DC1.3dmap.3dscen"},
                    {"output_path", "../test/test_data/DC1.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_DC2 =

            {
                    {"map_name",    "DC2"},
                    {"map_path",    "../test/test_data/DC2.3dmap"},
                    {"vis_path",    "../test/test_data/DC2.vis"},
                    {"block_path",  "../test/test_data/DC2.block"},
                    {"block_path_oc",  "../test/test_data/DC2_oc.block"},
                    {"minimum_block_width", "52"},
                    {"los_output_path", "../test/test_data/DC2-los.txt"},
                    {"config_path", "../test/test_data/DC2.3dmap.3dscen"},
                    {"output_path", "../test/test_data/DC2.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_EB1 =

            {
                    {"map_name",    "EB1"},
                    {"map_path",    "../test/test_data/EB1.3dmap"},
                    {"vis_path",    "../test/test_data/EB1.vis"},
                    {"block_path",  "../test/test_data/EB1.block"},
                    {"block_path_oc",  "../test/test_data/EB1_oc.block"},
                    {"minimum_block_width", "54"},
                    {"los_output_path", "../test/test_data/EB1-los.txt"},
                    {"config_path", "../test/test_data/EB1.3dmap.3dscen"},
                    {"output_path", "../test/test_data/EB1.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_EB2 =

            {
                    {"map_name",    "EB2"},
                    {"map_path",    "../test/test_data/EB2.3dmap"},
                    {"vis_path",    "../test/test_data/EB2.vis"},
                    {"block_path",  "../test/test_data/EB2.block"},
                    {"block_path_oc",  "../test/test_data/EB2_oc.block"},
                    {"minimum_block_width", "60"},
                    {"los_output_path", "../test/test_data/EB2-los.txt"},
                    {"config_path", "../test/test_data/EB2.3dmap.3dscen"},
                    {"output_path", "../test/test_data/EB2.txt"},
                    {"shrink_level", "5"}
            };

    SingleMapTestConfig<3> MapTestConfig_EC1 =

            {
                    {"map_name",    "EC1"},
                    {"map_path",    "../test/test_data/EC1.3dmap"},
                    {"vis_path",    "../test/test_data/EC1.vis"},
                    {"block_path",  "../test/test_data/EC1.block"},
                    {"block_path_oc",  "../test/test_data/EC1_oc.block"},
                    {"minimum_block_width", "37"},
                    {"los_output_path", "../test/test_data/EC1-los.txt"},
                    {"config_path", "../test/test_data/EC1.3dmap.3dscen"},
                    {"output_path", "../test/test_data/EC1.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_EC2 =

            {
                    {"map_name",    "EC2"},
                    {"map_path",    "../test/test_data/EC2.3dmap"},
                    {"vis_path",    "../test/test_data/EC2.vis"},
                    {"block_path",  "../test/test_data/EC2.block"},
                    {"block_path_oc",  "../test/test_data/EC2_oc.block"},
                    {"minimum_block_width", "40"}, // 33 pre
                    {"los_output_path", "../test/test_data/EC2-los.txt"},
                    {"config_path", "../test/test_data/EC2.3dmap.3dscen"},
                    {"output_path", "../test/test_data/EC2.txt"},
                    {"shrink_level", "1"}
            };



    // run out of storage space when init
    SingleMapTestConfig<3> MapTestConfig_Full4 =

            {
                    {"map_name",    "Full4"},
                    {"map_path",    "../test/test_data/Full4.3dmap"},
                    {"vis_path",    "../test/test_data/Full4.vis"},
                    {"block_path",  "../test/test_data/Full4.block"},
                    {"block_path_oc",  "../test/test_data/Full4_oc.block"},
                    {"minimum_block_width", "40"},
                    {"los_output_path", "../test/test_data/Full4-los.txt"},
                    {"config_path", "../test/test_data/Full4.3dmap.3dscen"},
                    {"output_path", "../test/test_data/Full4.txt"},
                    {"shrink_level", "1"}
            };


    RandomMapTestConfig<2> grid_2D_0 = {"grid_2D_0",
                                        {500, 500}, 10, 5,
                                        "../test/test_data/random_2d_0.rm",
                                        "../test/test_data/random_2d_0.block",
                                        "../test/test_data/random_2d_0.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_1 = {"grid_2D_1",
                                        {500, 500}, 10, 20,
    "../test/test_data/random_2d_1.rm",
    "../test/test_data/random_2d_1.block",
    "../test/test_data/random_2d_1.txt",
    3, 3};

    RandomMapTestConfig<2> grid_2D_2 = {"grid_2D_2",
                                        {500, 500}, 10, 40,
                                        "../test/test_data/grid_2D_2.rm",
                                        "../test/test_data/grid_2D_2.block",
                                        "../test/test_data/grid_2D_2.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_3 = {"grid_2D_3",
                                        {500, 500}, 10, 60,
                                        "../test/test_data/grid_2D_3.rm",
                                        "../test/test_data/grid_2D_3.block",
                                        "../test/test_data/grid_2D_3.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_4 = {"grid_2D_4",
                                        {500, 500}, 10, 80,
                                        "../test/test_data/grid_2D_4.rm",
                                        "../test/test_data/grid_2D_4.block",
                                        "../test/test_data/grid_2D_4.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_5 = {"grid_2D_5",
                                        {500, 500}, 10, 100,
                                        "../test/test_data/grid_2D_4.rm",
                                        "../test/test_data/grid_2D_4.block",
                                        "../test/test_data/grid_2D_4.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_6 = {"grid_2D_6",
                                        {500, 500}, 5, 60,
                                        "../test/test_data/grid_2D_6.rm",
                                        "../test/test_data/grid_2D_6.block",
                                        "../test/test_data/grid_2D_6.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_7 = {"grid_2D_7",
                                        {100, 100}, 5, 70,
                                        "../test/test_data/grid_2D_7.rm",
                                        "../test/test_data/grid_2D_7.block",
                                        "../test/test_data/grid_2D_7.txt",
                                        3, 3};

    // dimension length of map and block has merely no influence on the efficiency of JOB los check
    RandomMapTestConfig<2> grid_2D_8 = {"grid_2D_8",
                                        {100, 100}, 5, 80,
                                        "../test/test_data/grid_2D_8.rm",
                                        "../test/test_data/grid_2D_8.block",
                                        "../test/test_data/grid_2D_8.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_8_1 = {"grid_2D_8_1",
                                        {200, 200}, 5, 320,
                                        "../test/test_data/grid_2D_8_1.rm",
                                        "../test/test_data/grid_2D_8_1.block",
                                        "../test/test_data/grid_2D_8_1.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_8_2 = {"grid_2D_8_2",
                                          {300, 300}, 5, 720,
                                          "../test/test_data/grid_2D_8_2.rm",
                                          "../test/test_data/grid_2D_8_2.block",
                                          "../test/test_data/grid_2D_8_2.txt",
                                          3, 3};

    RandomMapTestConfig<2> grid_2D_8_3 = {"grid_2D_8_3",
                                          {400, 400}, 5, 1280,
                                          "../test/test_data/grid_2D_8_3.rm",
                                          "../test/test_data/grid_2D_8_3.block",
                                          "../test/test_data/grid_2D_8_3.txt",
                                          3, 3};

    RandomMapTestConfig<2> grid_2D_8_4 = {"grid_2D_8_4",
                                          {500, 500}, 5, 2000,
                                          "../test/test_data/grid_2D_8_4.rm",
                                          "../test/test_data/grid_2D_8_4.block",
                                          "../test/test_data/grid_2D_8_4.txt",
                                          3, 3};

    RandomMapTestConfig<2> grid_2D_8_5 = {"grid_2D_8_5",
                                          {600, 600}, 5, 2880,
                                          "../test/test_data/grid_2D_8_5.rm",
                                          "../test/test_data/grid_2D_8_5.block",
                                          "../test/test_data/grid_2D_8_5.txt",
                                          3, 3};

    RandomMapTestConfig<2> grid_2D_8_6 = {"grid_2D_8_6",
                                          {700, 700}, 5, 3920,
                                          "../test/test_data/grid_2D_8_6.rm",
                                          "../test/test_data/grid_2D_8_6.block",
                                          "../test/test_data/grid_2D_8_6.txt",
                                          3, 3};

    RandomMapTestConfig<3> grid_3D_1 = {"grid_3D_1",
                                        {100, 100, 100}, 5, 40,
                                        "../test/test_data/grid_3D_1.rm",
                                        "../test/test_data/grid_3D_1.block",
                                        "../test/test_data/grid_3D_1.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_2 = {"grid_3D_2",
                                        {100, 100, 100}, 5, 80,
                                        "../test/test_data/grid_3D_2.rm",
                                        "../test/test_data/grid_3D_2.block",
                                        "../test/test_data/grid_3D_2.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_3 = {"grid_3D_3",
                                        {100, 100, 100}, 5, 120,
                                        "../test/test_data/grid_3D_3.rm",
                                        "../test/test_data/grid_3D_3.block",
                                        "../test/test_data/grid_3D_3.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_4 = {"grid_3D_4",
                                        {100, 100, 100}, 5, 160,
                                        "../test/test_data/grid_3D_4.rm",
                                        "../test/test_data/grid_3D_4.block",
                                        "../test/test_data/grid_3D_4.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_5 = {"grid_3D_5",
                                        {100, 100, 100}, 5, 200,
                                        "../test/test_data/grid_3D_5.rm",
                                        "../test/test_data/grid_3D_5.block",
                                        "../test/test_data/grid_3D_5.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_6 = {"grid_3D_6",
                                        {100, 100, 100}, 5, 240,
                                        "../test/test_data/grid_3D_6.rm",
                                        "../test/test_data/grid_3D_6.block",
                                        "../test/test_data/grid_3D_6.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_7 = {"grid_3D_7",
                                        {100, 100, 100}, 5, 280,
                                        "../test/test_data/grid_3D_7.rm",
                                        "../test/test_data/grid_3D_7.block",
                                        "../test/test_data/grid_3D_7.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_8 = {"grid_3D_8",
                                        {100, 100, 100}, 5, 320,
                                        "../test/test_data/grid_3D_8.rm",
                                        "../test/test_data/grid_3D_8.block",
                                        "../test/test_data/grid_3D_8.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_8_1 = {"grid_3D_8_1",
                                        {150, 150, 150}, 5, 1080,
                                        "../test/test_data/grid_3D_8_1.rm",
                                        "../test/test_data/grid_3D_8_1.block",
                                        "../test/test_data/grid_3D_8_1.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_8_2 = {"grid_3D_8_2",
                                          {200, 200, 200}, 5, 2560,
                                          "../test/test_data/grid_3D_8_2.rm",
                                          "../test/test_data/grid_3D_8_2.block",
                                          "../test/test_data/grid_3D_8_2.txt",
                                          3, 3};

    RandomMapTestConfig<3> grid_3D_8_3 = {"grid_3D_8_3",
                                          {250, 250, 250}, 5, 5000,
                                          "../test/test_data/grid_3D_8_3.rm",
                                          "../test/test_data/grid_3D_8_3.block",
                                          "../test/test_data/grid_3D_8_3.txt",
                                          3, 3};

    RandomMapTestConfig<3> grid_3D_8_4 = {"grid_3D_8_4",
                                          {300, 300, 300}, 5, 8640,
                                          "../test/test_data/grid_3D_8_4.rm",
                                          "../test/test_data/grid_3D_8_4.block",
                                          "../test/test_data/grid_3D_8_4.txt",
                                          3, 3};


#define dimen_4d 50
#define cubic_width_4d 5
#define min_block_width_4d 2
#define shrink_level_4d 2

    RandomMapTestConfig<4> grid_4D_1 = {"grid_4D_1",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 60,
                                        "../test/test_data/grid_4D_1.rm",
                                        "../test/test_data/grid_4D_1.block",
                                        "../test/test_data/grid_4D_1.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_2 = {"grid_4D_2",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 120,
                                        "../test/test_data/grid_4D_2.rm",
                                        "../test/test_data/grid_4D_2.block",
                                        "../test/test_data/grid_4D_2.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_3 = {"grid_4D_3",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 180,
                                        "../test/test_data/grid_4D_3.rm",
                                        "../test/test_data/grid_4D_3.block",
                                        "../test/test_data/grid_4D_3.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_4 = {"grid_4D_4",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 240,
                                        "../test/test_data/grid_4D_4.rm",
                                        "../test/test_data/grid_4D_4.block",
                                        "../test/test_data/grid_4D_4.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_5 = {"grid_4D_5",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 300,
                                        "../test/test_data/grid_4D_5.rm",
                                        "../test/test_data/grid_4D_5.block",
                                        "../test/test_data/grid_4D_5.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_6 = {"grid_4D_6",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 360,
                                        "../test/test_data/grid_4D_6.rm",
                                        "../test/test_data/grid_4D_6.block",
                                        "../test/test_data/grid_4D_6.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_7 = {"grid_4D_7",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 420,
                                        "../test/test_data/grid_4D_7.rm",
                                        "../test/test_data/grid_4D_7.block",
                                        "../test/test_data/grid_4D_7.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_8 = {"grid_4D_8",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 480,
                                        "../test/test_data/grid_4D_8.rm",
                                        "../test/test_data/grid_4D_8.block",
                                        "../test/test_data/grid_4D_8.txt",
                                        min_block_width_4d, shrink_level_4d};

}
#endif //FREENAV_TEST_DATA_H
