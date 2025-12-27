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
    --enable-slope: 启用斜坡地形处理（默认开启）
    --disable-slope: 禁用斜坡地形处理
    --max-slope: 最大可通行坡度，默认0.3（约17°）
    --max-step-height: 最大可通行台阶高度，默认0.2米

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
    
    # 处理斜坡地形（自定义参数）
    python3 pcd_to_pgm.py map.pcd my_map --auto-height --max-slope 0.5 --max-step-height 0.3
    
    # 禁用斜坡处理（用于平坦地形）
    python3 pcd_to_pgm.py map.pcd my_map --auto-height --disable-slope
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

try:
    from scipy.spatial import cKDTree
    HAS_CKD = True
except Exception:
    HAS_CKD = False


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
                   origin_x=None, origin_y=None, auto_height=True, 
                   enable_slope_handling=True, max_slope=0.3, max_step_height=0.2):
    """
    将3D点云转换为2D占用栅格（支持斜坡地形处理）
    
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
        enable_slope_handling: 是否启用斜坡处理（默认True）
        max_slope: 最大可通行坡度（默认0.3，约17°）
        max_step_height: 最大可通行台阶高度（默认0.2m）
    
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
    
    # 获取xyz坐标（保留z值用于斜坡分析）
    xyz_points = filtered_points
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
    
    # 统计每个栅格的点数和高度信息
    occupancy_grid = np.zeros((height, width), dtype=np.int32)
    min_height_grid = np.full((height, width), np.inf, dtype=np.float32)  # 最低高度
    max_height_grid = np.full((height, width), -np.inf, dtype=np.float32)  # 最高高度
    mean_height_grid = np.zeros((height, width), dtype=np.float32)  # 平均高度
    height_sum_grid = np.zeros((height, width), dtype=np.float32)  # 高度总和（用于计算平均）
    
    # 将点投影到栅格并记录高度信息
    for point in xyz_points:
        grid_x = int((point[0] - origin_x) / resolution)
        grid_y = int((point[1] - origin_y) / resolution)
        
        if 0 <= grid_x < width and 0 <= grid_y < height:
            occupancy_grid[grid_y, grid_x] += 1
            height_sum_grid[grid_y, grid_x] += point[2]
            min_height_grid[grid_y, grid_x] = min(min_height_grid[grid_y, grid_x], point[2])
            max_height_grid[grid_y, grid_x] = max(max_height_grid[grid_y, grid_x], point[2])
    
    # 计算平均高度
    has_points = occupancy_grid > 0
    mean_height_grid[has_points] = height_sum_grid[has_points] / occupancy_grid[has_points]
    
    # 处理没有点的栅格
    min_height_grid[~has_points] = 0
    max_height_grid[~has_points] = 0
    
    # ============ 斜坡地形处理 ============
    if enable_slope_handling:
        print(f"\n启用斜坡地形处理:")
        print(f"  最大可通行坡度: {max_slope:.2f} (约 {np.degrees(np.arctan(max_slope)):.1f}°)")
        print(f"  最大台阶高度: {max_step_height:.2f} m")
        
        # 计算局部地面高度（使用最小高度网格的平滑版本）
        from scipy.ndimage import gaussian_filter, sobel, maximum_filter, minimum_filter, binary_dilation, grey_closing, binary_closing
        
        # 1. 预处理：填补稀疏点云造成的空洞
        # 稀疏点云会导致栅格中间有0值的“深坑”，这会产生巨大的虚假坡度
        # 使用灰度闭运算填补这些小坑（假设周围是平滑地形）
        # size=5 对应约 5*resolution (例如 0.25m) 的填补范围
        filled_height_grid = grey_closing(min_height_grid, size=5)
        
        # 2. 扩展有效点区域
        # 同样，稀疏点云会导致 has_points 也是断断续续的
        # 使用二值闭运算将断开的点连接起来，使坡面连通
        connected_points_mask = binary_closing(has_points, structure=np.ones((3,3)))
        
        # 3. 平滑地形
        # 使用填补后的高度图进行高斯平滑（减少噪声，增强坡面连通性）
        # 增大 sigma 以平滑整个坡面，使坡度计算更连续
        smooth_ground_height = gaussian_filter(filled_height_grid, sigma=1.5)
        
        # 计算坡度（使用Sobel算子）
        grad_x = sobel(smooth_ground_height, axis=1) / resolution
        grad_y = sobel(smooth_ground_height, axis=0) / resolution
        slope_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # 计算局部高度极差（形态学梯度），用于检测边缘
        # 在3x3邻域内计算最大高度差，这比单纯的梯度更能反映地形的陡峭边缘
        local_max = maximum_filter(smooth_ground_height, size=3)
        local_min = minimum_filter(smooth_ground_height, size=3)
        local_height_range = local_max - local_min
        
        # 识别陡峭边缘（高度变化剧烈的区域）
        # 这些通常是斜坡的侧边或台阶
        # 注意：这里使用 connected_points_mask 确保边缘检测覆盖填补后的区域
        steep_edges = connected_points_mask & (local_height_range > max_step_height)
        
        # 对边缘进行膨胀，将其转换为有一定宽度的不可通过区域
        # 增加 iterations=2 (约10cm宽度)，确保形成足够厚的"虚拟墙"，防止机器人从坡道边缘掉落
        steep_edges_dilated = binary_dilation(steep_edges, iterations=2)
        
        # 判断可通行性
        # 1. 在连通后的区域内
        # 2. 局部高度差小于阈值（不是陡峭边缘）
        # 3. 局部坡度小于最大坡度（缓坡可通行）
        # 4. 且不属于膨胀后的边缘区域（安全余量）
        traversable_mask = (
            connected_points_mask & 
            (local_height_range < max_step_height) & 
            (slope_magnitude < max_slope) &
            ~steep_edges_dilated
        )
        
        # 障碍物定义：
        # 1. 在连通区域内但不可通行的区域（太陡或太乱）
        # 2. 或者是识别出的陡峭边缘区域（包含膨胀的安全边界）
        obstacle_mask = (connected_points_mask & ~traversable_mask) | steep_edges_dilated
        
        # 更新 has_points 为连通后的掩码，以便后续统计正确
        has_points = connected_points_mask
        
        print(f"\n斜坡分析统计:")
        print(f"  总栅格数: {has_points.sum():7d}")
        print(f"  可通行栅格: {traversable_mask.sum():7d} ({100*traversable_mask.sum()/has_points.sum():5.2f}%)")
        print(f"  障碍物栅格: {obstacle_mask.sum():7d} ({100*obstacle_mask.sum()/has_points.sum():5.2f}%)")
        print(f"  平均坡度: {slope_magnitude[has_points].mean():.3f} (约 {np.degrees(np.arctan(slope_magnitude[has_points].mean())):.1f}°)")
        print(f"  最大坡度: {slope_magnitude[has_points].max():.3f} (约 {np.degrees(np.arctan(slope_magnitude[has_points].max())):.1f}°)")
        
    else:
        # 不启用斜坡处理时，使用简单的点密度阈值
        occupied_threshold = 1
        obstacle_mask = (occupancy_grid >= occupied_threshold)
    
    # 标记占用栅格
    grid[obstacle_mask] = 0  # 主要占用：值为0
    
    # 计算占用栅格的边界
    from scipy.ndimage import binary_dilation, distance_transform_edt
    
    # 膨胀占用区域以生成边界
    obstacle_dilated = binary_dilation(obstacle_mask, iterations=1)
    boundary_mask = obstacle_dilated & ~obstacle_mask
    
    # 边界和某些占用栅格标记为1
    grid[boundary_mask] = 1
    
    # 距离变换：计算每个点到最近障碍物的距离
    distances = distance_transform_edt(~obstacle_mask) * resolution
    
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
    parser.add_argument('--enable-slope', action='store_true', default=True,
                       help='启用斜坡地形处理（默认开启）')
    parser.add_argument('--disable-slope', action='store_false', dest='enable_slope',
                       help='禁用斜坡地形处理')
    parser.add_argument('--max-slope', type=float, default=0.3,
                       help='最大可通行坡度（默认0.3，约17°）')
    parser.add_argument('--max-step-height', type=float, default=0.2,
                       help='最大可通行台阶高度（米，默认0.2）')
    # Outlier removal options
    parser.add_argument('--remove-outliers', action='store_true', default=False,
                       help='启用离群噪声点去除（在高度过滤前执行）')
    parser.add_argument('--outlier-method', type=str, default=None,
                       choices=['statistical', 'radius'],
                       help="离群点去除方法：'statistical' 使用 Open3D 的统计滤波；'radius' 使用半径邻域计数")
    parser.add_argument('--outlier-nb-neighbors', type=int, default=20,
                       help='统计滤波时的邻域大小（statistical）')
    parser.add_argument('--outlier-std-ratio', type=float, default=2.0,
                       help='统计滤波的标准差比（statistical）')
    parser.add_argument('--outlier-radius', type=float, default=0.5,
                       help='半径方法的邻域半径（米，radius）')
    parser.add_argument('--outlier-min-neighbors', type=int, default=3,
                       help='半径方法中一个点被视为非噪声所需的最小邻居数（包括自身）')
    
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

    # 可选：去除离群噪声点（在高度过滤之前）
    if args.remove_outliers:
        method = args.outlier_method
        # 自动选择方法：优先 statistical（若 open3d 可用），否则 radius
        if method is None:
            method = 'statistical' if HAS_OPEN3D else 'radius'

        def remove_outliers_statistical(pts, nb_neighbors=20, std_ratio=2.0):
            if not HAS_OPEN3D:
                print('统计滤波需要 open3d，但未检测到 open3d，跳过统计滤波')
                return pts
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
            filtered = np.asarray(cl.points)
            print(f"统计滤波: 原始点数={len(pts)}, 剔除后点数={len(filtered)}")
            return filtered

        def remove_outliers_radius(pts, radius=0.5, min_neighbors=3):
            if not HAS_CKD:
                print('半径滤波需要 SciPy (cKDTree)，但未检测到，跳过半径滤波')
                return pts
            tree = cKDTree(pts)
            # query_ball_point 返回每个点在半径内的邻居索引列表
            neighbors = tree.query_ball_point(pts, r=radius)
            counts = np.array([len(n) for n in neighbors])
            mask = counts >= min_neighbors
            filtered = pts[mask]
            print(f"半径滤波 (r={radius}, min_n={min_neighbors}): 原始点数={len(pts)}, 剔除后点数={len(filtered)}")
            return filtered

        if method == 'statistical':
            points = remove_outliers_statistical(points,
                                                nb_neighbors=args.outlier_nb_neighbors,
                                                std_ratio=args.outlier_std_ratio)
        else:
            points = remove_outliers_radius(points,
                                           radius=args.outlier_radius,
                                           min_neighbors=args.outlier_min_neighbors)
    
    # 转换为2D网格
    grid, origin = pcd_to_2d_grid(
        points,
        resolution=args.resolution,
        height_min=args.height_min,
        height_max=args.height_max,
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        auto_height=args.auto_height,
        enable_slope_handling=args.enable_slope,
        max_slope=args.max_slope,
        max_step_height=args.max_step_height
    )
    
    if grid is None:
        print("地图转换失败")
        sys.exit(1)
    
    # 保存地图
    save_map(grid, origin, args.resolution, args.output_name)


if __name__ == '__main__':
    main()
