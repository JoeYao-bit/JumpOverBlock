import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker
import os
from PIL import Image
import matplotlib.image as mpimg
import math

def loadDataFromfile(file_path):
    data_list = list()
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                splited_line = line.split()

                if splited_line[0] == "COMPARE":
                                
                    new_data = CompareData()
                    
                    new_data.dim                  = int(splited_line  [1])
                    new_data.colli_ratio          = float(splited_line[2])
                    new_data.raw_time_cost        = float(splited_line[3])
                    new_data.SBT_time_cost        = float(splited_line[4])
                    new_data.total_index_of_space = int(splited_line  [5])
                    new_data.occ_ratio            = float(splited_line[6])
                    new_data.dimension_length     = int(splited_line  [7])

                    data_list.append(new_data)

                elif splited_line[0] == "SBT":
                    new_data = SBTData()
                    
                    new_data.dim                   = int(splited_line  [1])
                    new_data.init_time_cost        = float(splited_line[2])
                    new_data.update_time_cost      = float(splited_line[3])
                    new_data.total_index_of_space  = int(splited_line  [4])
                    new_data.occ_ratio             = float(splited_line[5])
                    new_data.max_obs_move_distance = int(splited_line  [6])
                    new_data.dimension_length      = int(splited_line  [7])
                    data_list.append(new_data)

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
    raw_time_cost = 0.
    SBT_time_cost = 0.
    total_index_of_space = 0
    occ_ratio = 0.
    dimension_length = 0 


class SBTData:
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

def drawCompareData(all_data, dim):
    map_data = {} # dimelength and data occ ratio, dimension_length
    for a_data in all_data:
        if type(a_data).__name__ != "CompareData":
            continue
        if a_data.dim != dim:
            continue
        if a_data.dimension_length not in map_data:
            map_data[a_data.dimension_length] = dict()
            map_data[a_data.dimension_length]["colli_ratio"] = list()
            map_data[a_data.dimension_length]["acc_ratio"] = list()
        map_data[a_data.dimension_length]["colli_ratio"].append(a_data.colli_ratio)
        map_data[a_data.dimension_length]["acc_ratio"].append(a_data.SBT_time_cost / a_data.raw_time_cost)

    fig=plt.figure(figsize=(5,4)) #添加绘图框
    ax = plt.axes()
    plt.ylabel("Acc Ratio", fontsize = 13)
    plt.xlabel("Collision Ratio", fontsize = 13)    
    for map_key, map_value in map_data.items():
        x = map_value["colli_ratio"]
        y = map_value["acc_ratio"]
        scatter = ax.scatter(x, y, marker='.')
        
        coefficients = np.polyfit(x, y, deg=1)
        trend_line = np.poly1d(coefficients)
        plt.plot(x, trend_line(x), label='width='+str(map_key))
        
    plt.legend(ncol=2)    
    # plt.show()     
    save_path = "../test/pic/"
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
    save_path = save_path + "dim_"+str(dim)+"_compare_summary"    
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)        



def drawSBTInitData(all_data, dim):
    map_data = {} # dimelength and data occ ratio, dimension_length
    for a_data in all_data:
        if type(a_data).__name__ != "SBTData":
            continue
        if a_data.dim != dim:
            continue
        if a_data.dimension_length not in map_data:
            map_data[a_data.dimension_length] = dict()
            map_data[a_data.dimension_length]["occ_ratio"] = list()
            map_data[a_data.dimension_length]["init_time_cost"] = list()
        map_data[a_data.dimension_length]["occ_ratio"].append(a_data.occ_ratio)
        map_data[a_data.dimension_length]["init_time_cost"].append(a_data.init_time_cost)

    fig=plt.figure(figsize=(5,4)) #添加绘图框
    ax = plt.axes()
    plt.ylabel("Init Time Cost (ms)", fontsize = 13)
    plt.xlabel("Occ Ratio", fontsize = 13)    
    for map_key, map_value in map_data.items():
        x = map_value["occ_ratio"]
        y = map_value["init_time_cost"]
        scatter = ax.scatter(x, y, marker='.')
        
        coefficients = np.polyfit(x, y, deg=1)
        trend_line = np.poly1d(coefficients)
        plt.plot(x, trend_line(x), label='width='+str(map_key))
        
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


def drawSBTUpdateData(all_data, dim):
    map_data = {} # dimelength and data occ ratio, dimension_length
    for a_data in all_data:
        if type(a_data).__name__ != "SBTData":
            continue
        if a_data.dim != dim:
            continue
        if a_data.dimension_length not in map_data:
            map_data[a_data.dimension_length] = dict()
            map_data[a_data.dimension_length]["occ_ratio"] = list()
            map_data[a_data.dimension_length]["update_time_cost"] = list()
        map_data[a_data.dimension_length]["occ_ratio"].append(a_data.occ_ratio)
        map_data[a_data.dimension_length]["update_time_cost"].append(a_data.update_time_cost)

    fig=plt.figure(figsize=(5,4)) #添加绘图框
    ax = plt.axes()
    plt.ylabel("Update Time Cost(ms)", fontsize = 13)
    plt.xlabel("Occ Ratio", fontsize = 13)    
    for map_key, map_value in map_data.items():
        x = map_value["occ_ratio"]
        y = map_value["update_time_cost"]
        scatter = ax.scatter(x, y, marker='.')
        
        coefficients = np.polyfit(x, y, deg=1)
        trend_line = np.poly1d(coefficients)
        plt.plot(x, trend_line(x), label='width='+str(map_key))
        
    plt.legend(ncol=2)    
    # plt.show()     
    save_path = "../test/pic/"
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
    save_path = save_path + "dim_"+str(dim)+"_update_summary"    
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)        



dim_color_map = {
    2:"green",
    3:"blue",
}

file_path = "../test/SBT_LOS.txt"

all_compare_data = loadDataFromfile(file_path)

drawSBTInitData(all_compare_data, 3)
drawSBTInitData(all_compare_data, 2)

drawSBTUpdateData(all_compare_data, 3)
drawSBTUpdateData(all_compare_data, 2)

drawCompareData(all_compare_data, 3)
drawCompareData(all_compare_data, 2)


