#!/usr/bin/env python3
"""
PCD点云地图转换为2D占用栅格地图（PGM格式） - Nav2 导航系统适配版

用法:
    python3 pcd_to_pgm.py input.pcd output_map_name [options]

参数:
    input_pcd: 输入的PCD点云文件路径
    output_map_name: 输出地图的名称（不含扩展名）
    
选项:
    --resolution: 地图分辨率，默认0.05米/像素（= 20像素/米）
    --height-min: 考虑的最小高度，默认自动检测
    --height-max: 考虑的最大高度，默认自动检测
    --auto-height: 启用自动检测高度范围（推荐）
    --origin-x: 地图原点X坐标，默认自动计算
    --origin-y: 地图原点Y坐标，默认自动计算

输出:
    output_map_name.pgm: 二进制栅格地图（PGM格式）
    output_map_name.yaml: Nav2配置文件

像素值对应（参考RMUC格式）:
    0 - 占用空间（黑色）
    1 - 占用边界
    253 - 自由空间（过渡区域/灰色）
    254 - 自由空间（主要/浅灰）
    255 - 未知空间（白色）

示例:
    # 自动检测高度范围（推荐）
    python3 pcd_to_pgm.py map.pcd my_map --auto-height
    
    # 手动指定高度范围
    python3 pcd_to_pgm.py map.pcd my_map --height-min 0.1 --height-max 2.0
    
    # 自定义分辨率
    python3 pcd_to_pgm.py map.pcd my_map --auto-height --resolution 0.025
"""

import numpy as np
import yaml
import argparse
import sys
from pathlib import Path

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("警告: 未安装open3d，尝试使用pcl库")
    
try:
    import pcl
    HAS_PCL = True
except ImportError:
    HAS_PCL = False


def load_pcd_open3d(pcd_path):
    """使用Open3D加载PCD文件"""
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    return points


def load_pcd_pcl(pcd_path):
    """使用python-pcl加载PCD文件"""
    cloud = pcl.load(pcd_path)
    points = np.array(cloud)
    return points


def load_pcd_manual(pcd_path):
    """手动解析PCD文件（备用方案）"""
    points = []
    with open(pcd_path, 'r') as f:
        data_started = False
        for line in f:
            if line.startswith('DATA'):
                data_started = True
                continue
            if data_started:
                try:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        points.append([x, y, z])
                except:
                    continue
    return np.array(points)


def analyze_height_distribution(points):
    """
    分析点云的高度(Z轴)分布
    
    返回建议的高度范围和统计信息
    """
    z_values = points[:, 2]
    z_min = z_values.min()
    z_max = z_values.max()
    z_mean = z_values.mean()
    z_std = z_values.std()
    
    analysis = {
        'z_min': z_min,
        'z_max': z_max,
        'z_mean': z_mean,
        'z_std': z_std,
        'z_range': z_max - z_min,
        'suggested_min': z_min - z_std * 0.5,  # 下界 = min - 0.5*std
        'suggested_max': z_max + z_std * 0.5,  # 上界 = max + 0.5*std
    }
    
    print("\n" + "=" * 60)
    print("点云高度(Z轴)分析")
    print("=" * 60)
    print(f"Z 轴最小值: {z_min:.6f} m")
    print(f"Z 轴最大值: {z_max:.6f} m")
    print(f"Z 轴范围: {analysis['z_range']:.6f} m")
    print(f"Z 轴平均值: {z_mean:.6f} m")
    print(f"Z 轴标准差: {z_std:.6f} m")
    print(f"\n建议的高度范围:")
    print(f"  最小值: {analysis['suggested_min']:.6f} m")
    print(f"  最大值: {analysis['suggested_max']:.6f} m")
    print("=" * 60 + "\n")
    
    return analysis


def pcd_to_2d_grid(points, resolution=0.05, height_min=None, height_max=None, 
                   origin_x=None, origin_y=None, auto_height=True):
    """
    将3D点云转换为2D占用栅格（参考RMUC格式）
    
    像素值对应关系（参考RMUC.pgm）：
        0 - 占用空间（障碍物）
        1 - 占用边界
        253 - 自由空间（过渡区域）
        254 - 自由空间（主要）
        255 - 未知空间
    
    参数:
        points: Nx3的numpy数组
        resolution: 地图分辨率（米/像素，默认0.05）
        height_min: 考虑的最小高度（None表示自动）
        height_max: 考虑的最大高度（None表示自动）
        origin_x, origin_y: 地图原点坐标
        auto_height: 如果高度范围为None，是否自动检测
    
    返回:
        grid: 2D占用栅格（像素值0-255）
        origin: [x, y, 0] 地图原点
    """
    # 自动检测高度范围
    if auto_height and (height_min is None or height_max is None):
        analysis = analyze_height_distribution(points)
        if height_min is None:
            height_min = analysis['suggested_min']
        if height_max is None:
            height_max = analysis['suggested_max']
        print(f"使用自动检测的高度范围: [{height_min:.6f}, {height_max:.6f}] m\n")
    
    # 设置默认值（如果仍然为None）
    if height_min is None:
        height_min = 0.1  # 略高于地面，避免地面反射
    if height_max is None:
        height_max = 2.0
    
    # 过滤高度范围内的点
    mask = (points[:, 2] >= height_min) & (points[:, 2] <= height_max)
    filtered_points = points[mask]
    
    if len(filtered_points) == 0:
        print(f"✗ 错误：在高度范围 [{height_min:.6f}, {height_max:.6f}] m 内没有找到点")
        print(f"\n提示：")
        print(f"  • 请检查PCD文件的Z轴坐标单位（可能是厘米或毫米，不是米）")
        print(f"  • 或使用参数 --auto-height 自动检测高度范围")
        analysis = analyze_height_distribution(points)
        return None, None
    
    print(f"原始点云数量: {len(points)}")
    print(f"高度过滤后点云数量: {len(filtered_points)}")
    print(f"点云范围:")
    print(f"  X: [{filtered_points[:, 0].min():.2f}, {filtered_points[:, 0].max():.2f}]")
    print(f"  Y: [{filtered_points[:, 1].min():.2f}, {filtered_points[:, 1].max():.2f}]")
    print(f"  Z: [{filtered_points[:, 2].min():.2f}, {filtered_points[:, 2].max():.2f}]")
    
    # 获取xy坐标
    xy_points = filtered_points[:, :2]
    
    # 计算地图边界
    min_x, min_y = xy_points.min(axis=0)
    max_x, max_y = xy_points.max(axis=0)
    
    # 设置原点（如果未指定，使用最小值并留出边距）
    margin = 1.0  # 1米边距
    if origin_x is None:
        origin_x = min_x - margin
    if origin_y is None:
        origin_y = min_y - margin
    
    # 计算栅格大小
    width = int((max_x - origin_x + 2 * margin) / resolution) + 1
    height = int((max_y - origin_y + 2 * margin) / resolution) + 1
    
    print(f"\n地图参数:")
    print(f"  分辨率: {resolution} m/pixel (= {1/resolution:.1f} pixels/meter)")
    print(f"  尺寸: {width} x {height} pixels")
    print(f"  覆盖范围: {width * resolution:.2f}m x {height * resolution:.2f}m")
    print(f"  原点: ({origin_x:.2f}, {origin_y:.2f}, 0)")
    
    # 创建空栅格（参考RMUC格式）
    # 初始化为255（未知空间）
    grid = np.ones((height, width), dtype=np.uint8) * 255
    
    # 统计每个栅格的点数
    occupancy_grid = np.zeros((height, width), dtype=np.int32)
    occupancy_type = np.zeros((height, width), dtype=np.uint8)  # 占用类型
    
    # 将点投影到栅格
    for point in xy_points:
        grid_x = int((point[0] - origin_x) / resolution)
        grid_y = int((point[1] - origin_y) / resolution)
        
        if 0 <= grid_x < width and 0 <= grid_y < height:
            occupancy_grid[grid_y, grid_x] += 1
    
    # 设置栅格值（参考RMUC.pgm的像素值分布）
    # 阈值：至少1个点标记为占用
    occupied_threshold = 1
    
    # 标记占用栅格
    occupied_mask = (occupancy_grid >= occupied_threshold)
    grid[occupied_mask] = 0  # 主要占用：值为0
    
    # 计算占用栅格的边界
    from scipy.ndimage import binary_dilation, distance_transform_edt
    
    # 膨胀占用区域以生成边界
    occupied_dilated = binary_dilation(occupied_mask, iterations=1)
    boundary_mask = occupied_dilated & ~occupied_mask
    
    # 边界和某些占用栅格标记为1
    grid[boundary_mask] = 1
    
    # 距离变换：计算每个点到最近障碍物的距离
    distances = distance_transform_edt(~occupied_mask) * resolution
    
    # 设置自由空间：
    # 靠近障碍物的自由空间为253（过渡区域）
    # 远离障碍物的自由空间为254（主要自由空间）
    near_obstacle_dist = 0.2  # 20cm内为过渡区域
    
    near_obstacle = (grid == 255) & (distances < near_obstacle_dist) & (distances > 0)
    grid[near_obstacle] = 253  # 过渡区域
    
    far_from_obstacle = (grid == 255) & (distances >= near_obstacle_dist)
    grid[far_from_obstacle] = 254  # 自由空间
    
    print(f"\n栅格统计:")
    print(f"  占用栅格 (0): {np.sum(grid == 0):7d} ({100*np.sum(grid == 0)/grid.size:5.2f}%)")
    print(f"  占用边界 (1): {np.sum(grid == 1):7d} ({100*np.sum(grid == 1)/grid.size:5.2f}%)")
    print(f"  自由过渡(253): {np.sum(grid == 253):7d} ({100*np.sum(grid == 253)/grid.size:5.2f}%)")
    print(f"  自由空间(254): {np.sum(grid == 254):7d} ({100*np.sum(grid == 254)/grid.size:5.2f}%)")
    print(f"  未知空间(255): {np.sum(grid == 255):7d} ({100*np.sum(grid == 255)/grid.size:5.2f}%)")
    
    # 翻转Y轴（图像坐标系与地图坐标系不同）
    grid = np.flipud(grid)
    
    return grid, [origin_x, origin_y, 0.0]


def save_map(grid, origin, resolution, output_path):
    """
    保存地图为PGM和YAML文件（参考RMUC格式）
    
    参数:
        grid: 2D占用栅格
        origin: 地图原点 [x, y, theta]
        resolution: 分辨率
        output_path: 输出文件路径（不含扩展名）
    
    YAML参数说明（参考RMUC.yaml）:
        occupied_thresh: 0.65 - 像素值 > 此值*255 时判定为占用
        free_thresh: 0.25 - 像素值 < 此值*255 时判定为自由
    """
    output_path = Path(output_path)
    
    # 保存PGM文件
    pgm_path = output_path.with_suffix('.pgm')
    with open(pgm_path, 'wb') as f:
        # PGM头部（P5 = 二进制格式）
        f.write(b'P5\n')
        f.write(f'{grid.shape[1]} {grid.shape[0]}\n'.encode())  # width height
        f.write(b'255\n')  # 最大像素值
        # 像素数据
        f.write(grid.tobytes())
    
    print(f"\n已保存PGM地图: {pgm_path}")
    print(f"  尺寸: {grid.shape[1]} x {grid.shape[0]} pixels")
    
    # 保存YAML配置文件（参考RMUC.yaml格式）
    yaml_path = output_path.with_suffix('.yaml')
    
    # 参考RMUC.yaml的参数
    # occupied_thresh: 0.65 意味着像素值 > 0.65*255=166 时为占用
    # free_thresh: 0.25 意味着像素值 < 0.25*255=64 时为自由
    # 这对应我们的像素值规划：
    #   0-1: 占用 (0/255=0, 1/255≈0.004 < 0.25)
    #   253-254: 自由 (253/255≈0.99, 254/255≈0.996 > 0.65)
    #   255: 未知
    
    map_config = {
        'image': pgm_path.name,
        'mode': 'trinary',                    # 三元模式（占用/自由/未知）
        'resolution': float(resolution),
        'origin': [float(x) for x in origin],
        'negate': 0,                          # 不反转黑白
        'occupied_thresh': 0.65,              # 占用阈值（参考RMUC）
        'free_thresh': 0.25                   # 自由阈值（参考RMUC）
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(map_config, f, 
                 default_flow_style=False,
                 sort_keys=False,
                 allow_unicode=True)
    
    print(f"已保存YAML配置: {yaml_path}")
    print(f"\nYAML配置内容:")
    print(f"  image: {pgm_path.name}")
    print(f"  mode: trinary")
    print(f"  resolution: {resolution} m/pixel")
    print(f"  origin: {origin}")
    print(f"  negate: 0")
    print(f"  occupied_thresh: 0.65 (像素值 > {int(0.65*255)} 为占用)")
    print(f"  free_thresh: 0.25 (像素值 < {int(0.25*255)} 为自由)")
    
    print(f"\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print(f"✓ 地图转换完成！")
    print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print(f"\n使用方法:")
    print(f"  ros2 launch slash_nav2 bringup_real.launch.py map:={yaml_path.absolute()}")
    print(f"\n或者在启动前验证地图:")
    print(f"  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={yaml_path.absolute()}")
    print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")


def main():
    parser = argparse.ArgumentParser(
        description='将PCD点云地图转换为Nav2可用的2D占用栅格地图',
        epilog="""
参考格式：RMUC.yaml/RMUC.pgm

像素值对应关系:
  0-1   占用空间（障碍物）
  253   自由空间（过渡/灰色）
  254   自由空间（主要/浅灰）
  255   未知空间（白色）

示例用法:
  # 自动检测高度范围（推荐）
  python3 pcd_to_pgm.py map.pcd output_map --auto-height
  
  # 手动指定参数
  python3 pcd_to_pgm.py map.pcd output_map --height-min 0.1 --height-max 2.0 --resolution 0.05
  
  # 启动导航系统
  ros2 launch slash_nav2 bringup_all.launch.py map:=$(pwd)/output_map.yaml
        """
    )
    parser.add_argument('input_pcd', type=str, help='输入的PCD文件路径')
    parser.add_argument('output_name', type=str, help='输出地图名称（不含扩展名）')
    parser.add_argument('--resolution', type=float, default=0.05, 
                       help='地图分辨率（米/像素），默认0.05 (= 20 pixels/meter)')
    parser.add_argument('--height-min', type=float, default=None, 
                       help='考虑的最小高度（米），默认自动检测')
    parser.add_argument('--height-max', type=float, default=None, 
                       help='考虑的最大高度（米），默认自动检测')
    parser.add_argument('--auto-height', action='store_true', 
                       help='启用自动检测最优的高度范围（推荐）')
    parser.add_argument('--origin-x', type=float, default=None, 
                       help='地图原点X坐标，默认自动计算')
    parser.add_argument('--origin-y', type=float, default=None, 
                       help='地图原点Y坐标，默认自动计算')
    
    args = parser.parse_args()
    
    # 处理自动高度检测
    if args.auto_height:
        args.height_min = None
        args.height_max = None
    
    # 检查输入文件
    input_path = Path(args.input_pcd)
    if not input_path.exists():
        print(f"错误：找不到输入文件 {input_path}")
        sys.exit(1)
    
    print(f"正在加载PCD文件: {input_path}")
    
    # 尝试不同的加载方法
    points = None
    if HAS_OPEN3D:
        try:
            points = load_pcd_open3d(str(input_path))
            print("使用Open3D加载成功")
        except Exception as e:
            print(f"Open3D加载失败: {e}")
    
    if points is None and HAS_PCL:
        try:
            points = load_pcd_pcl(str(input_path))
            print("使用python-pcl加载成功")
        except Exception as e:
            print(f"python-pcl加载失败: {e}")
    
    if points is None:
        try:
            points = load_pcd_manual(str(input_path))
            print("使用手动解析加载成功")
        except Exception as e:
            print(f"手动解析失败: {e}")
            print("\n请安装以下库之一:")
            print("  pip install open3d")
            print("  或")
            print("  pip install python-pcl")
            sys.exit(1)
    
    if len(points) == 0:
        print("错误：点云为空")
        sys.exit(1)
    
    # 转换为2D网格
    grid, origin = pcd_to_2d_grid(
        points,
        resolution=args.resolution,
        height_min=args.height_min,
        height_max=args.height_max,
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        auto_height=args.auto_height
    )
    
    if grid is None:
        print("地图转换失败")
        sys.exit(1)
    
    # 保存地图
    save_map(grid, origin, args.resolution, args.output_name)


if __name__ == '__main__':
    main()
