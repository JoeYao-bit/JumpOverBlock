import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker
import os
from PIL import Image
import matplotlib.image as mpimg
import math
from scipy.ndimage import uniform_filter1d

def loadDataFromfile(file_path):
    print("file_path = ", file_path)
    data_list = list()
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                # print(line)
                splited_line = line.split()
                if splited_line[0] == "COMPARE":
                    new_data = CompareData()
                    
                    new_data.dim                  = int(splited_line  [1])
                    new_data.colli_ratio          = float(splited_line[2])
                    new_data.RAW_LOS_time_cost    = float(splited_line[3])
                    new_data.SBT_RAW_time_cost    = float(splited_line[4])
                    new_data.SBT_NEW_time_cost    = float(splited_line[5])

                    new_data.RAW_VISIT_PT         = float(splited_line[6])
                    new_data.SBT_RAW_VISIT_PT     = float(splited_line[7])
                    new_data.SBT_NEW_VISIT_PT     = float(splited_line[8])

                    new_data.SBT_RAW_VISIT_BC     = float(splited_line[9])
                    new_data.SBT_NEW_VISIT_BC     = float(splited_line[10])

                    new_data.total_index_of_space = int(splited_line  [11])
                    new_data.occ_ratio            = float(splited_line[12])
                    new_data.dimension_length     = int(splited_line  [13])

                    data_list.append(new_data)

                    # print("raw/raw sbt/new sbt visited pt = ", new_data.RAW_VISIT_PT, " / ", new_data.SBT_RAW_VISIT_PT, " / ", new_data.SBT_NEW_VISIT_PT)

                    # print("raw sbt/new sbt visited bc = ", new_data.SBT_RAW_VISIT_BC, " / ", new_data.SBT_NEW_VISIT_BC)

                elif splited_line[0] == "SBT" or splited_line[0] == "SBT_RAW":
                    new_data = SBTData()
                    
                    new_data.name                  = splited_line[0] 
                    new_data.dim                   = int(splited_line  [1])
                    new_data.init_time_cost        = float(splited_line[2])
                    new_data.update_time_cost      = float(splited_line[3])
                    new_data.total_index_of_space  = int(splited_line  [4])
                    new_data.occ_ratio             = float(splited_line[5])
                    new_data.max_obs_move_distance = int(splited_line  [6])
                    new_data.dimension_length      = int(splited_line  [7])
                    data_list.append(new_data)

                    # print(new_data)

    except Exception as e:            
        print(e)             
    return data_list

# std::stringstream ss;
# ss << N << " "  // Dimension
#    << (float)occ_count / success_count << " " // occ ratio
#    << sum_1/(double)success_count << " " // mean raw time cost
#    << sum_2/(double)success_count // mean SBT time cost
#    << getTotalIndexOfSpace<N>(temp_dim) << " " // total index of space
#    << (float)dynamic_obstacles.occ_pt_count_ / space_size << " " // ratio of occ grid
#    << printDimInfo<N>(temp_dim) << " " // dimension length
#    ;

class CompareData:
    dim = 0
    colli_ratio = 0.
    RAW_LOS_time_cost = 0.
    SBT_RAW_time_cost = 0.
    SBT_NEW_time_cost = 0.

    RAW_VISIT_PT = 0.
    SBT_RAW_VISIT_PT = 0.
    SBT_NEW_VISIT_PT = 0.

    SBT_RAW_VISIT_BC= 0.
    SBT_NEW_VISIT_BC = 0.
    total_index_of_space = 0
    occ_ratio = 0.
    dimension_length = 0 


class SBTData:
    name = ""
    dim = 0
    init_time_cost = 0.
    update_time_cost = 0.
    total_index_of_space = 0
    occ_ratio = 0.
    dimension_length = 0 
    max_obs_move_distance = 0

# def drawData3D(all_data, dim):
#     x1 = []
#     y1 = []
#     z1 = []
#     for a_data in all_data:
#         if a_data.dim == dim:
#             x1.append(a_data.occ_ratio)
#             y1.append(a_data.dimension_length)
#             z1.append(a_data.SBT_time_cost / a_data.raw_time_cost)

#     fig=plt.figure(figsize=(5,3.5)) #添加绘图框

#     ax = plt.axes(projection='3d')
#     ax.set_xlabel('Occ ratio', fontsize = 13)
#     ax.set_ylabel('Width', fontsize = 13)
#     ax.set_zlabel('Acc ratio', fontsize = 13)
    
#     scatter = ax.scatter(x1, y1, z1, 
#                 c=dim_color_map[dim], marker='.', label='dim='+str(dim))
#     plt.show()     

def drawCompareTimeCost(all_data, dim):
    map_data = dict() # dimelength and data occ ratio, dimension_length
    for a_data in all_data:
        if type(a_data).__name__ != "CompareData":
            continue
        if a_data.dim != dim:
            continue

        if "SBT_RAW" not in map_data:
            map_data["SBT_RAW"] = dict()

        if "SBT" not in map_data:
            map_data["SBT"] = dict()    
        
        if "RAW" not in map_data:
            map_data["RAW"] = dict()    

        if a_data.dimension_length not in map_data["SBT"]:
            map_data["RAW"][a_data.dimension_length] = dict()
            map_data["RAW"][a_data.dimension_length]["colli_ratio"] = list()
            map_data["RAW"][a_data.dimension_length]["time_cost"] = list()

        if a_data.dimension_length not in map_data["SBT_RAW"]:
            map_data["SBT_RAW"][a_data.dimension_length] = dict()
            map_data["SBT_RAW"][a_data.dimension_length]["colli_ratio"] = list()
            map_data["SBT_RAW"][a_data.dimension_length]["time_cost"] = list()

        if a_data.dimension_length not in map_data["SBT"]:
            map_data["SBT"][a_data.dimension_length] = dict()
            map_data["SBT"][a_data.dimension_length]["colli_ratio"] = list()
            map_data["SBT"][a_data.dimension_length]["time_cost"] = list()

        map_data["SBT_RAW"][a_data.dimension_length]["colli_ratio"].append(a_data.colli_ratio)
        map_data["SBT_RAW"][a_data.dimension_length]["time_cost"].append(a_data.SBT_RAW_time_cost)

        map_data["SBT"][a_data.dimension_length]["colli_ratio"].append(a_data.colli_ratio)
        map_data["SBT"][a_data.dimension_length]["time_cost"].append(a_data.SBT_NEW_time_cost)

        map_data["RAW"][a_data.dimension_length]["colli_ratio"].append(a_data.colli_ratio)
        map_data["RAW"][a_data.dimension_length]["time_cost"].append(a_data.RAW_LOS_time_cost)

    fig=plt.figure(figsize=(5,4)) #添加绘图框
    ax = plt.axes()
    plt.ylabel("Time cost(us)", fontsize = 13)
    plt.xlabel("Collision Ratio", fontsize = 13)    

    for type_key, type_value in map_data.items():
        for map_key, map_value in map_data[type_key].items():
            x = map_value["colli_ratio"]
            y = map_value["time_cost"]
            scatter = ax.scatter(x, y, marker='.')

            # 按x排序
            sort_idx = np.argsort(x)

            x_sorted = list()
            y_sorted = list()
            for idx in sort_idx:
                x_sorted.append(x[idx])
                y_sorted.append(y[idx])
            # 计算移动平均
            window_size = math.ceil(len(x)/5)  # 控制平滑程度
            y_smooth = uniform_filter1d(y_sorted, size=window_size) # window_size must be integer
            plt.plot(x_sorted, y_smooth, label=label_map_of_type[type_key]+'_width='+str(map_key))
            
            # coefficients = np.polyfit(x, y, deg=1)
            # trend_line = np.poly1d(coefficients)
            # plt.plot(x, trend_line(x), label=label_map_of_type[type_key]+'_'+'width='+str(map_key))
        
    plt.legend(ncol=2)    
    # plt.show()     
    save_path = "../test/pic/"
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
    save_path = save_path + "dim_"+str(dim)+"_timecost_compare_summary"    
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)        


def printStatistic(all_data, dim):
    map_data = dict() # dimelength and data occ ratio, dimension_length
    for a_data in all_data:
        if type(a_data).__name__ != "CompareData":
            continue
        if a_data.dim != dim:
            continue

        if "SBT_RAW" not in map_data:
            map_data["SBT_RAW"] = dict()

        if "SBT" not in map_data:
            map_data["SBT"] = dict()    
        
        if "RAW" not in map_data:
            map_data["RAW"] = dict()    

        if a_data.dimension_length not in map_data["RAW"]:
            map_data["RAW"] = dict()
            map_data["RAW"]["visited_pt"] = list()
            map_data["RAW"]["time_cost"] = list()

        if a_data.dimension_length not in map_data["SBT_RAW"]:
            map_data["SBT_RAW"] = dict()
            map_data["SBT_RAW"]["visited_pt"] = list()
            map_data["SBT_RAW"]["visited_bc"] = list()
            map_data["SBT_RAW"]["time_cost"] = list()

        if a_data.dimension_length not in map_data["SBT"]:
            map_data["SBT"] = dict()
            map_data["SBT"]["visited_pt"] = list()
            map_data["SBT"]["visited_bc"] = list()
            map_data["SBT"]["time_cost"] = list()


        map_data["RAW"]["visited_pt"].append(a_data.RAW_VISIT_PT)
        map_data["RAW"]["time_cost"].append(a_data.RAW_LOS_time_cost) 

        map_data["SBT_RAW"]["visited_pt"].append(a_data.SBT_RAW_VISIT_PT)
        map_data["SBT_RAW"]["visited_bc"].append(a_data.SBT_RAW_VISIT_BC)
        map_data["SBT_RAW"]["time_cost"].append(a_data.SBT_RAW_time_cost)

        map_data["SBT"]["visited_pt"].append(a_data.SBT_NEW_VISIT_PT)
        map_data["SBT"]["visited_bc"].append(a_data.SBT_NEW_VISIT_BC)
        map_data["SBT"]["time_cost"].append(a_data.SBT_NEW_time_cost)

    print(dim,"D raw_LOS/raw_SBT/new_SBT time cost(us) = ", np.mean(map_data["RAW"]["time_cost"]), "/", np.mean(map_data["SBT_RAW"]["time_cost"]), "/", np.mean(map_data["SBT"]["time_cost"]))

    print(dim, "D raw_LOS/raw_SBT/new_SBT visited pt = ", np.mean(map_data["RAW"]["visited_pt"]), "/", np.mean(map_data["SBT_RAW"]["visited_pt"]), "/", np.mean(map_data["SBT"]["visited_pt"]))

    print(dim, "D raw_SBT/new_SBT visited bc = ", np.mean(map_data["SBT_RAW"]["visited_bc"]), "/", np.mean(map_data["SBT"]["visited_bc"]))
      

def drawSBTInitData(all_data, dim):
    map_data = dict() # dimelength and data occ ratio, dimension_length
    for a_data in all_data:
        if type(a_data).__name__ != "SBTData":
            continue
        if a_data.dim != dim:
            continue

        if a_data.name not in map_data:
            map_data[a_data.name] = dict()
        if a_data.dimension_length not in map_data[a_data.name]:
            map_data[a_data.name][a_data.dimension_length] = dict()
            map_data[a_data.name][a_data.dimension_length]["occ_ratio"] = list()
            map_data[a_data.name][a_data.dimension_length]["init_time_cost"] = list()

        map_data[a_data.name][a_data.dimension_length]["occ_ratio"].append(a_data.occ_ratio)
        map_data[a_data.name][a_data.dimension_length]["init_time_cost"].append(a_data.init_time_cost)

    fig=plt.figure(figsize=(5,4)) #添加绘图框
    ax = plt.axes()
    plt.ylabel("Init Time Cost (ms)", fontsize = 13)
    plt.xlabel("Occ Ratio", fontsize = 13)    
    for type_key, type_value in map_data.items():
        for map_key, map_value in map_data[type_key].items():
            x = map_value["occ_ratio"]
            y = map_value["init_time_cost"]
            scatter = ax.scatter(x, y, marker='.')

            # 按x排序
            sort_idx = np.argsort(x)

            x_sorted = list()
            y_sorted = list()
            for idx in sort_idx:
                x_sorted.append(x[idx])
                y_sorted.append(y[idx])
            # 计算移动平均
            window_size = math.ceil(len(x)/5)  # 控制平滑程度
            y_smooth = uniform_filter1d(y_sorted, size=window_size) # window_size must be integer
            plt.plot(x_sorted, y_smooth, label=label_map_of_type[type_key]+'_width='+str(map_key))

            # coefficients = np.polyfit(x, y, deg=1)
            # trend_line = np.poly1d(coefficients)
            # plt.plot(x, trend_line(x), label=label_map_of_type[type_key]+'_width='+str(map_key))
        
    plt.legend(ncol=2)    
    # plt.show()     
    save_path = "../test/pic/"
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
    save_path = save_path + "dim_"+str(dim)+"_init_summary"    
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)        


def drawSBTUpdateData(all_data, dim, max_obs_move_dist):
    map_data = dict() # dimelength and data occ ratio, dimension_length
    for a_data in all_data:
        if type(a_data).__name__ != "SBTData":
            continue
        if a_data.dim != dim:
            continue
        if a_data.max_obs_move_distance != max_obs_move_dist:
            continue        

        if a_data.name not in map_data:
            map_data[a_data.name] = dict()

        if a_data.dimension_length not in map_data[a_data.name]:
            map_data[a_data.name][a_data.dimension_length] = dict()
            map_data[a_data.name][a_data.dimension_length]["occ_ratio"] = list()
            map_data[a_data.name][a_data.dimension_length]["update_time_cost"] = list()

        map_data[a_data.name][a_data.dimension_length]["occ_ratio"].append(a_data.occ_ratio)
        map_data[a_data.name][a_data.dimension_length]["update_time_cost"].append(a_data.update_time_cost)

    fig=plt.figure(figsize=(5,4)) #添加绘图框
    ax = plt.axes()
    plt.ylabel("Update Time Cost(ms)", fontsize = 13)
    plt.xlabel("Occ Ratio", fontsize = 13)    

    for type_key, type_value in map_data.items():
        for map_key, map_value in map_data[type_key].items():
            x = map_value["occ_ratio"]
            y = map_value["update_time_cost"]
            scatter = ax.scatter(x, y, marker='.')

            # 按x排序
            sort_idx = np.argsort(x)

            x_sorted = list()
            y_sorted = list()
            for idx in sort_idx:
                x_sorted.append(x[idx])
                y_sorted.append(y[idx])
            # 计算移动平均
            window_size = math.ceil(len(x)/5)  # 控制平滑程度
            y_smooth = uniform_filter1d(y_sorted, size=window_size) # window_size must be integer
            plt.plot(x_sorted, y_smooth, label=label_map_of_type[type_key]+'_width='+str(map_key))

            # print('x/y size = ', len(x), " / ", len(y))
            # coefficients = np.polyfit(x, y, deg=1)
            # trend_line = np.poly1d(coefficients)
            # plt.plot(x, trend_line(x), label=label_map_of_type[type_key]+'_width='+str(map_key))
        
    plt.legend(ncol=2)    
    # plt.show()     
    save_path = "../test/pic/"
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
    save_path = save_path + "dim_"+str(dim)+"_dist_"+ str(max_obs_move_dist) +"_update_summary"    
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)        



dim_color_map = {
    2:"green",
    3:"blue",
}

label_map_of_type = {
    "RAW":"RAW",
    "SBT":"SBT",
    "SBT_RAW":"QUAD/OCT"
}

file_path = "../test/SBT_LOS.txt"

all_compare_data = loadDataFromfile(file_path)

drawSBTInitData(all_compare_data, 2)
for max_dist in [1, 4, 16, 0]:
    drawSBTUpdateData(all_compare_data, 2, max_dist)

drawCompareTimeCost(all_compare_data, 2)
printStatistic(all_compare_data, 2)

# drawSBTInitData(all_compare_data, 3)
# for max_dist in [1, 4, 16, 0]:
#     drawSBTUpdateData(all_compare_data, 3, max_dist)

# drawCompareTimeCost(all_compare_data, 3)
# printStatistic(all_compare_data, 3)

# # 生成示例数据
# np.random.seed(42)
# x = np.linspace(0, 10, 500)
# y = np.sin(x) + np.random.normal(0, 0.5, len(x))

# # 按x排序
# sort_idx = np.argsort(x)
# x_sorted, y_sorted = x[sort_idx], y[sort_idx]

# # 计算移动平均
# window_size = 30  # 控制平滑程度
# y_smooth = uniform_filter1d(y_sorted, size=window_size)

# plt.figure(figsize=(12, 6))
# plt.scatter(x, y, alpha=0.3, label='Raw Data', s=10)
# plt.plot(x_sorted, y_smooth, 'r-', lw=3, label=f'Moving Average (window={window_size})')
# plt.title('Moving Average Smoothing')
# plt.legend()
# plt.grid(True)
# plt.show()