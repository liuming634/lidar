# 基于PCD→PNG映射的场地配置方案

用PCD点云生成灰度高程PNG作为2D地图，通过聚类代表点的XY对应到PNG像素位置，取区域灰度平均反算高度，配合配置文件做范围矫正，替代手动测量和硬编码。

---

## 整体数据流

```
LiDAR点云 → 预处理 → PCD静态地图 (field.pcd)
                              │
               ┌──────────────┴──────────────┐
               ▼                              ▼
      欧几里得聚类 (动态物体)         生成灰度高程PNG
               │                     (投影到2D俯视图)
               │                              │
               ▼                              ▼
      聚类代表点 (x, y, z)          高程图 (2_elevation_gray256.png)
               │                     2916×1505, 8bit灰度
               │                              │
               └────────── XY映射 ────────────┘
                              │
                              ▼
                  取代表点周围窗口内的有效灰度值
                  计算平均灰度 G_avg
                              │
                              ▼
                  反算高度: H = G_avg × 0.008490 - 0.187180
                              │
                              ▼
                  用PCD真实 z 值做范围矫正
                  输出: 高度数据 + 长宽尺寸
```

---

## 高程PNG规格

| 项目 | 数值 |
|------|------|
| 图像尺寸 | 2916 × 1505 像素 |
| 灰度位深 | 8 bit (0~255) |
| 最低高度 | -0.187180 m |
| 最高高度 | 1.969253 m |
| 每级灰度 | 8.49 mm/级 |
| 无数据标记 | 255 (白色) |

### 灰度-高度换算公式

灰度值 G (0 ≤ G ≤ 254) → 高度 H：

```
H = G / 254 × 2.156433 - 0.187180
H = G × 0.008490 - 0.187180
```

高度 H → 灰度值 G：

```
G = (H + 0.187180) / 0.008490
```

### 配置文件 (meta.yaml)

```yaml
elevation_max_m: 1.969253
elevation_min_m: -0.187180
formula: H = G / 254 × (max - min) + min
pixel_max: 254
pixel_min: 0
pixel_nodata: 255
width: 2916
height: 1505
```

> 换场地时只需重新生成PNG和meta.yaml，调整高度范围参数即可。

---

## 实现步骤

### Step 1：生成PNG高程图

从PCD点云生成灰度高程PNG：
1. 加载PCD点云 `(x, y, z)`
2. 将点云投影到2D俯视图网格，每个网格取 `z_max` 作为高度
3. 将高度线性映射到灰度值 0~254，保存为PNG
4. 同时生成 meta.yaml 记录高度范围

### Step 2：聚类提取代表点

LiDAR动态点云经欧几里得聚类后，每个cluster计算质心 `(x, y, z)`：
- `(x, y)` 用于映射到PNG像素位置
- `z` 作为真实高度用于后续矫正

### Step 3：XY→UV坐标映射

将聚类代表点的场地坐标 `(x, y)` 转换到PNG像素坐标 `(u, v)`：
- 需要知道PNG的分辨率（米/像素）
- 需要知道原点 `(0, 0)` 对应的像素位置
- y轴翻转（场地坐标y↑对应图像v↓）

### Step 4：区域灰度采样与高度反算

在 `(u, v)` 周围取窗口（如5×5像素）：
1. 收集窗口内所有 G ≠ 255 的有效像素灰度值
2. 计算平均灰度值 G_avg
3. 用公式反算高度: `H = G_avg × 0.008490 - 0.187180`

### Step 5：高度矫正

```
对第 i 个聚类物体:
  真实高度 z_i (来自PCD点云)
  灰度推算高度 H_i (来自PNG灰度平均)

矫正偏移: offset = mean(z_i - H_i)
矫正后高度: H_corrected = H + offset
```

---

## 代码示例

```python
import cv2
import numpy as np

# 加载灰度高程图
img = cv2.imread('config/2_elevation_gray256.png', cv2.IMREAD_GRAYSCALE)

# 场地坐标 (x, y) → 像素坐标 (u, v)
def xy_to_uv(x, y, resolution, origin_x, origin_y):
    u = int((x - origin_x) / resolution)
    v = int((origin_y - y) / resolution)  # y轴翻转
    return u, v

# 在 (u,v) 周围取窗口平均灰度并反算高度
def get_avg_height(img, u, v, window_size=5):
    half = window_size // 2
    roi = img[max(0, v-half):v+half+1, max(0, u-half):u+half+1]
    valid = roi[roi != 255].astype(np.float32)
    if len(valid) == 0:
        return None
    G_avg = np.mean(valid)
    H = G_avg * 0.008490 - 0.187180
    return H

# 对每个聚类代表点查询高度
for cluster_x, cluster_y, cluster_z in representative_points:
    u, v = xy_to_uv(cluster_x, cluster_y, res, ox, oy)
    H = get_avg_height(img, u, v)
    if H is not None:
        print(f"({cluster_x:.3f}, {cluster_y:.3f}) → "
              f"灰度推算: {H:.4f}m, PCD真实: {cluster_z:.4f}m")
```

---

## 配置文件说明

| 文件 | 用途 | 是否换场地就改 |
|------|------|---------------|
| `config/field.pcd` | PCD静态地图 | ✅ 重新扫描 |
| `config/2_elevation_gray256.png` | 灰度高程图 | ✅ 重新生成 |
| `config/*_meta.yaml` | 高度范围参数 | ✅ 需确认范围 |
| `config/camera_params.yaml` | 相机内参 | ❌ 换相机才改 |
| `config/out_matrix.yaml` | 相机外参 | ❌ 拆装相机才改 |

---

## 与传统方式对比

| 方式 | 精度 | 效率 | 说明 |
|------|------|------|------|
| 卷尺手动量 | ±5~10cm | 慢，需进场 | 不便于频繁操作 |
| 原硬编码配置 | 依赖手动测量 | 改代码，编译 | 灵活性差 |
| **PCD→PNG映射(新)** | **±0.5~3cm** | **快，一次生成** | **换场地只需重跑脚本** |





---

## 换场地需改参数总表（按必须改 / 可保留 / 可废弃）

### 🔴 必须改 —— 换场地必须重新配置，不改跑不起来

| 类别 | 文件 | 行 | 内容 | 原因 |
|------|------|----|------|------|
| **文件路径** | `localization.cpp` | 33 | `"config/RM2025.pcd"` | 场地先验地图，必换 |
| | `dynamic_cloud.cpp` | 21 | `"config/RM2025.pcd"` | 同上 |
| | `debug_map.cpp` | 36 | `"config/RM2025.png"` | 场地底图，必换 |
| | `resolve.cpp` | 16 | `"config/RM2025.png"` | 同上 |
| **场地尺寸** | `kalman_filter.cpp` | 200-205 | `28 - x`, `15 - y` | 坐标翻转用 |
| | `resolve.cpp` | 102-136 | `/ 28`, `/ 15` | 地图绘制 |
| | `debug_map.cpp` | 50 | `Size(28 * 25, 15 * 25)` | 底图大小 |
| | `debug_map.cpp` | 76-77,87-88 | `/ 28`, `/ 15` | 点位绘制 |
| | `debug_map.cpp` | 114-115,125-126 | `28 - x`, `15 - y` | 坐标翻转 |
| **裁剪范围** | `localization.cpp` | 113 | `x:5~30, y:-10~8, z<7` | 雷达位置不同需重调 |
| | `maps/map.yaml` | 1 | `RM2025.png` | 同上 |
| | `radar_utils.cpp` | 118 | `"./config/RM2025_Points.yaml"` | **高程图方案可废弃** |
| **相机外参** | `radar_utils.cpp` | 21,54,169,192 | `"./config/out_matrix.yaml"` | 重新标定 |
| | `calibrate.cpp` | 191 | `"./config/out_matrix.yaml"` | 重新标定 |
| | `dynamic_cloud.cpp` | 210-211 | `x:3~28, y:0~15, z:0~1.4` | 同上 |
| | `dynamic_cloud.cpp` | 212-216 | 排除区 5 个值 | 角部障碍物不同 |
| **透视参考点** | `radar_utils.cpp` | 94-97,104-107 | `(12,-6),(16,-6)`等4点 | 决定透视变换基准 |
| **飞镖区域** | `dynamic_cloud.cpp` | 154-156,217-219 | 飞镖空间立方体 6 

| 类别 | 文件 | 行 | 内容 | 默认值 | 说明 |
|------|------|----|------|--------|------|
| 聚类 | `cluster.cpp` | 42 | setClust个值 | **高程图方案可废弃** |
| **无人机区域** | `dynamic_cloud.cpp` | 160-176,220-222 | 3个区域共 ~18 个值 | **高程图方案可废弃** |
| **英雄阈值** | `debug_map.cpp` | 166,169-171,185 | `28-8.668`, `28-20.3`等 | 新场地布局不同，**建议整体删除** |

---

### 🟡 可保留 —— 一般不用改，场地特殊时再调erTolerance | 0.25m | 只要车尺寸差不多就不用改 |
| | `cluster.cpp` | 43-44 | 簇大小范围 | 5~1000 | 同上 |
| KF匹配 | `filter_plus.h` | 44 | detect_r | 1.0m | 匹配半径 |
| | `filter_plus.h` | 46 | car_max_speed | 2.5m/s | RM车辆限速 |
| | `filter_plus.h` | 38 | delete_time | 2.0s | 目标超时删除 |
| | `filter_plus.h` | 181 | TIME_THRESHOLD | 1.0s | 相机匹配时间窗 |
| 配准 | `localization.cpp` | 121 | VoxelGrid叶大小 | 0.1m | 计算量/精度平衡 |
| | `localization.cpp` | 155 | GICP 收敛阈值 | < 0.2 | |
| | `localization.cpp` | 189 | 累积帧数 | 20 | |
| | `localization.cpp` | 194 | 球面栅格步长 | 0.1° | |
| 动态点 | `dynamic_cloud.cpp` | 27 | 体素下采样 | 0.1f | |
| | `dynamic_cloud.cpp` | 230 | KD-tree距离阈值 | 0.1 | |
| | `dynamic_cloud.cpp` | 232 | 累积帧数 | 3 | |
| | `dynamic_cloud.h` | 29 | accumulate_time | 3 | |
| 报警 | `dynamic_cloud.cpp` | 273 | 飞镖点数阈值 | > 5 | |
| | `dynamic_cloud.cpp` | 296-304 | 无人机点数阈值 | > 40 | |
| 高度 | `radar_utils.cpp` | 9 | ARMOR_HEIGHT | 0.15m | 车底盘高度基准 |
| 显示 | `detect.cpp` | 349 | 显示图大小 | 1536×1125 | 相机分辨率 |

---

### 🟢 可废弃 —— 高程图方案替代后不再需要

| 文件 | 行 | 内容 | 替代方案 |
|------|----|------|---------|
| `config/RM2025_Points.yaml` | 全文 | 6个区域的3D多边形顶点 | 高程图直接采样，不用区域划分 |
| `radar_utils.cpp` | 36-48 | 6个区域对象创建+Height赋值 | 高程图查表 |
| `radar_utils.cpp` | 75-76 | 高度超0.79m的回退坐标 (19.322,-1.915) | 高程图给出正确高度不存在回退 |
| `radar_utils.cpp` | 81-89 | `get_height()` 函数（pointPolygonTest） | 改由高程图查高度 |
| `dynamic_cloud.cpp` | 153-177 | `dart_cloud_filter` / `fly_*_filter` 全部 | 新场地布局空间规则需重写 |
| `debug_map.cpp` | 158-192 | `hero_count1/2` 进退场检测 | 你们不需要，直接删 |
| `calibrate.h` | 37-41 | 5个特征点世界坐标 | 只是标定界面的默认值，每个场地自己重新选点 |

