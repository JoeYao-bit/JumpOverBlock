import struct
import numpy as np
import cv2
import yaml
import os

def parse_bonn_gfs(gfs_path, output_dir="."):
    """解析波恩大学 .gfs 文件并生成 ROS 2 兼容地图"""
    with open(gfs_path, 'rb') as f:
        # 读取文件头
        header = f.read(4)
        if header != b'GFS\n':
            raise ValueError("无效的 .gfs 文件格式")
        
        # 读取粒子数
        num_particles = struct.unpack('I', f.read(4))[0]
        print(f"找到 {num_particles} 个粒子")
        
        # 读取最优粒子索引
        best_idx = struct.unpack('I', f.read(4))[0]
        
        # 定位到最优粒子数据
        f.seek(24 + 132 * best_idx)  # 24字节头 + 132字节/粒子
        
        # 读取地图元数据
        map_size_x, map_size_y = struct.unpack('II', f.read(8))
        origin_x, origin_y = struct.unpack('dd', f.read(16))
        resolution = struct.unpack('d', f.read(8))[0]
        
        # 读取地图数据
        map_data = np.frombuffer(f.read(map_size_x * map_size_y), dtype=np.int8)
        map_grid = map_data.reshape((map_size_y, map_size_x))
    
    # 转换为 ROS 2 地图格式
    # ROS 2 使用：0-100 表示占用概率，-1 表示未知
    # 转换为图像：0=障碍(黑), 255=空闲(白), 205=未知(灰)
    image_data = np.zeros_like(map_grid, dtype=np.uint8)
    image_data[map_grid == 100] = 0      # 障碍物 - 黑色
    image_data[map_grid == 0] = 255      # 空闲区域 - 白色
    image_data[map_grid == -1] = 205     # 未知区域 - 灰色
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 保存PGM图像
    pgm_path = os.path.join(output_dir, "map.pgm")
    cv2.imwrite(pgm_path, image_data)
    
    # 生成YAML配置文件 (ROS 2兼容)
    yaml_content = {
        'image': 'map.pgm',
        'resolution': float(resolution),
        'origin': [float(origin_x), float(origin_y), 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'negate': 0
    }
    
    yaml_path = os.path.join(output_dir, "map.yaml")
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_content, f)
    
    print(f"地图已保存至: {os.path.abspath(output_dir)}")
    print(f"分辨率: {resolution} m/像素")
    print(f"原点坐标: [{origin_x}, {origin_y}]")

# 使用示例
parse_bonn_gfs("/home/yaozhuo/Downloads/csail.corrected.gfs", "/home/yaozhuo/Downloads/")