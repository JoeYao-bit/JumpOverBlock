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
                                
                new_data = LineData()
                
                new_data.dim                  = int(splited_line[0])
                new_data.colli_ratio            = float(splited_line[1])
                new_data.raw_time_cost        = float(splited_line[2])
                new_data.SBT_time_cost        = float(splited_line[3])
                new_data.total_index_of_space = int(splited_line[4])
                new_data.occ_ratio            = float(splited_line[5])
                new_data.dimension_length     = int(splited_line[6])

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

class LineData:
    dim = 0
    colli_ratio = 0.
    raw_time_cost = 0.
    SBT_time_cost = 0.
    total_index_of_space = 0
    occ_ratio = 0.
    dimension_length = 0 


def drawData3D(all_data, dim):
    x1 = []
    y1 = []
    z1 = []
    for a_data in all_data:
        if a_data.dim == dim:
            x1.append(a_data.occ_ratio)
            y1.append(a_data.dimension_length)
            z1.append(a_data.SBT_time_cost / a_data.raw_time_cost)

    fig=plt.figure(figsize=(5,3.5)) #添加绘图框

    ax = plt.axes(projection='3d')
    ax.set_xlabel('Occ ratio', fontsize = 13)
    ax.set_ylabel('Width', fontsize = 13)
    ax.set_zlabel('Acc ratio', fontsize = 13)
    
    scatter = ax.scatter(x1, y1, z1, 
                c=dim_color_map[dim], marker='.', label='dim='+str(dim))
    plt.show()     

def drawData(all_data, dim):

    map_data = {} # dimelength and data occ ratio, dimension_length
    for a_data in all_data:

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
    #ax.set_xlabel('Occ Ratio', fontsize = 13)
    #ax.set_ylabel('Acc Ratio', fontsize = 13)

    plt.ylabel("Acc Ratio", fontsize = 13)
    plt.xlabel("Collision Ratio", fontsize = 13)

    #ax.set_zlabel('Acc ratio', fontsize = 13)
    
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
    save_path = save_path + "dim_"+str(dim)+"_summary"    
    plt.savefig(save_path, dpi = 200)#, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)      


# plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[splited_method_name[0]], markersize=10, label=map_key+"/"+splited_method_name[0], linewidth=temp_width, elinewidth=4, capsize=4, markerfacecolor='none')

def drawData3D(all_data, dim):
    x1 = []
    y1 = []
    z1 = []
    for a_data in all_data:
        if a_data.dim == dim:
            x1.append(a_data.occ_ratio)
            y1.append(a_data.dimension_length)
            z1.append(a_data.SBT_time_cost / a_data.raw_time_cost)

    fig=plt.figure(figsize=(5,3.5)) #添加绘图框

    ax = plt.axes(projection='3d')
    ax.set_xlabel('Occ ratio', fontsize = 13)
    ax.set_ylabel('Width', fontsize = 13)
    ax.set_zlabel('Acc ratio', fontsize = 13)
    
    scatter = ax.scatter(x1, y1, z1, 
                c=dim_color_map[dim], marker='.', label='dim='+str(dim))
    plt.show()     

# # 第一组数据
# x1 = [1, 2, 3, 4]
# y1 = [2, 4, 6, 8]
# plt.scatter(x1, y1, c='blue', label='Group A')

# # 第二组数据
# x2 = [1, 3, 5, 7]
# y2 = [1, 2, 3, 4]
# plt.scatter(x2, y2, c='red', marker='s', label='Group B')

# plt.title("Multiple Scatter Plots")
# plt.legend()  # 显示图例
# plt.show()

dim_color_map = {
    2:"green",
    3:"blue",
}

file_path = "../test/SBT_LOS.txt"

all_data = loadDataFromfile(file_path)

drawData(all_data, 3)