# PCD 场景配置框架

用雷达扫描的 PCD 静态地图自动生成场地配置，替代手动测量和硬编码。

---

## 整体数据流

```
                     PCD 处理脚本
  ┌────────────┐    ┌──────────────────────────┐
  │ field.pcd  │───→│ scripts/pcd_to_config.py │
  └────────────┘    │                          │
                    │  ① RANSAC 地面分割       │──→ field_size_x/y
                    │  ② 非地面 DBSCAN 聚类    │──→ 区域多边形 (x,y)
                    │  ③ 每个 cluster z 均值   │──→ height
                    │  ④ 俯视图渲染             │──→ field_map.png
                    └──────────────────────────┘

                        运行时
  ┌──────────────┐    ┌─────────────────────────┐
  │ field_geom   │───→│ radar_utils (parser)    │
  │ .yaml        │    │  get_height()           │ ← polygonTest 归属
  │              │    │  get_2d()               │ ← 透视变换
  │ scene_rules  │───→│                         │
  │ .yaml        │    │  3D 场地坐标 (x,y)      │
  └──────────────┘    └─────────────────────────┘
      读
  ┌──────┴──────┐
  │ resolve     │ ← field_size_x/y + field_map.png
  │ debug_map   │ ← field_size_x/y + field_map.png
  │ dynamic_cl  │ ← field_bounds + 功能区
  └─────────────┘
```

---

## 数据采集方式

### 推荐：静态扫描（不需要 SLAM）

把雷达架在场地角落 2~3m 高处（三脚架或桌子），固定不动录点云。

```
Livox Mid-70 在 28m 处的 FOV 覆盖宽度 ≈ 39m
→ 一个角落就能覆盖整个 28m × 15m 场地
```

**步骤：**
1. 雷达架高，清场
2. 录 15~20 秒点云（多帧累积增加密度）
3. 保存为 PCD

### 更优：SLAM 绕场一圈建图（以后可学）

```
推着雷达绕场地走一圈
↓
Livox-SLAM / FAST-LIO2 拼接
↓
每个区域多角度覆盖，边界更密更准
```

静态扫描精度已经够用，SLAM 后续学习即可。

---

## 精度分析

### Mid-70 在 28m 场地内的实测精度

| 参数 | 精度 | 说明 |
|------|------|------|
| 场地尺寸 (x/y) | ±3cm | 地面点云 min/max 统计 |
| 区域高度 (z 均值) | ±0.5cm | 几千个点的统计均值 |
| 凸包顶点 | ±3~5cm | 取决于边缘点密度，多帧累积可改善 |

### 现有 RM2025.pcd 实际数据分析

```
X: -0.58 ~ 28.58  (跨度 29.2m，完整覆盖)
Y: -0.02 ~ 15.02  (跨度 15.0m，完整覆盖)

高度分布（每个高度层数万点，峰谷清晰分离）：
  0.00m 附近: 357,000 点  → 地面
  0.15m 附近:  13,793 点  → 要塞
  0.20m 附近:  52,450 点  → 道路
  0.30m 附近:  35,612 点  → 中场线
  0.60m 附近:   9,623 点  → 增益区
```

每个高度层都有**几千到几万个点**，z 值的统计均值非常稳定。实测证明这种方案可行。

### 误差对比

| 方式 | 误差 |
|------|------|
| 卷尺手动量 | ±5~10cm |
| PCD 凸包顶点 | ±3~5cm |
| PCD z 均值 | ±0.5cm |

---

## PCD 处理流程（两遍处理）

```
第一遍：提取场地尺寸
  ┌─────────────────────────────┐
  │ RANSAC 平面拟合 → 地面点   │
  │ 对地面点算 min/max         │
  │ → field_size_x, field_size_y│
  │ → ground_z                 │
  └─────────────────────────────┘

第二遍：提取区域
  ┌─────────────────────────────┐
  │ 非地面点 (z > ground_z+阈值) │
  │ → DBSCAN 聚类               │
  │ → 每个 cluster:             │
  │    ├ 凸包 → 多边形顶点 (x,y) │
  │    ├ z 均值 → height        │
  │    └ 按位置匹配语义名称     │
  └─────────────────────────────┘
```

为什么要分两遍：先只取地面点算边界，排除高台和噪点对尺寸的影响。

---

## 坐标对齐

雷达建图坐标系 (PCD 原点) 和 rm_frame（相机世界坐标系）可能不重合。

### 对齐方法

**方法一：雷达放在 rm_frame 原点**
把雷达架在场地角点（rm_frame 定义的原点），PCD 坐标 = rm_frame 坐标，不需要变换。

**方法二：手动标一个偏移**
在场地选一个已知点（比如己方要塞角点），在 PCD 中取该点坐标，和 rm_frame 下的坐标算平移量。

```
T = (x_rm - x_pcd, y_rm - y_pcd, 0)
PCD → rm_frame: 每个点 + T
```

---

## 配置文件

### field_geom.yaml（自动生成）

替换现有的 `RM2025_Points.yaml`，OpenCV FileStorage 格式：

```yaml
%YAML:1.0
---
field_size_x: 28.0      # 场地 X 方向长度
field_size_y: 15.0      # 场地 Y 方向宽度

Middle_Line:
  height: 0.3
  points:
    - { x: 9.767, y: -13.0 }
    - { x: 10.168, y: -5.529 }
    # ...

Left_Road:
  height: 0.2
  points:
    # ...

Right_Road:
  height: 0.2
  points:
    # ...

Enemy_Buff:
  height: 0.6
  points:
    # ...

Self_Fortress:
  height: 0.15
  points:
    # ...

Enemy_Fortress:
  height: 0.15
  points:
    # ...
```

每个点只有 (x, y)，高度统一用 `height` 字段。

### scene_rules.yaml（手动配置，一次配好）

```yaml
%YAML:1.0
---
# 透视变换参考点（4个3D点）
perspective_ref_points:
  - { x: 12, y: -6, z: 0.0 }
  - { x: 16, y: -6, z: 0.0 }
  - { x: 16, y: -8, z: 0.0 }
  - { x: 12, y: -8, z: 0.0 }

armor_height: 0.15              # 装甲板离地高度
height_threshold: 0.79          # 超出此高度返回固定点
fallback_point: { x: 19.322, y: -1.915 }

# 飞镖检测区
dart_zone:
  x_min: 27.22; x_max: 27.41
  y_min: 3.925; y_max: 4.525
  z_min: 1.713; z_max: 2.472
  threshold: 5

# 无人机三级预警区
fly_safe_zone:
  x_min: 25.225; x_max: 27.5
  y_min: 0.2; y_max: 2.2
  z_min: 1.7; z_max: 3.0
  threshold: 40

fly_warn_zone:
  x_min: 19.83; x_max: 25.3
  y_min: 0.2; y_max: 4.556
  z_min: 1.7; z_max: 3.0
  threshold: 40

fly_alarm_zone:
  x_min: 13.0; x_max: 20.5
  y_min: 0.2; y_max: 4.556
  z_min: 1.7; z_max: 3.0
  threshold: 40

# dynamic_cloud 场地过滤边界
field_bounds:
  x_min: 3.0; x_max: 28.0
  y_min: 0.0; y_max: 15.0
  z_min: 0.0; z_max: 1.4
  exclude_regions:
    - type: rect
      x_min: 25.0; x_max: 28.0
      y_min: 0.0; y_max: 5.0
    - type: rotated_rect
      center: { x: 21.5, y: -6.5 }
      size: { w: 2.9, h: 0.9 }

# 英雄进退场检测阈值
hero_detection:
  base_threshold: 8.668
  forward_x_min: 20.3
  forward_x_max: 25.075
  forward_y_min: 0.0
  forward_y_max: 10.3
```

---

## 2D→3D 映射管线

```
相机 2D 点 (u, v)
      ↓
get_height() ──→ 遍历 field_geom.yaml 的区域，判断 (u, v) 投影
      │            落在哪个多边形内 → 读该区域的 height
      ↓
得到 height
      ↓
get_2d() ──→ 用 scene_rules.yaml 的 4 个参考点 + height
      │       做透视变换
      ↓
场地 3D 坐标 (x, y) → 卡尔曼滤波
```

| 部分 | 原来 | 改成 |
|------|------|------|
| 区域多边形 | `RM2025_Points.yaml` 手填 (x,y,z) | `field_geom.yaml` (x,y) + height |
| 区域高度 | `radar_utils.cpp` 硬编码 | `field_geom.yaml` 读 |
| 透视参考点 | 硬编码 (12,-6)~(16,-8) | `scene_rules.yaml` 读 |
| 高度阈值 / 回退点 | 0.79, (19.322,-1.915) 硬编码 | `scene_rules.yaml` 读 |

---

## PCD 处理脚本

`scripts/pcd_to_config.py`，输入 `config/field.pcd`，输出 `field_geom.yaml` + `field_map.png`：

```
PCD 点云
  │
  ├─ 第一遍：RANSAC 地面拟合
  │    └─ 地面点 X/Y min/max → field_size_x, field_size_y
  │    └─ 地面 z 均值 → ground_z
  │
  ├─ 第二遍：非地面点 DBSCAN 聚类
  │    └─ 每个 cluster:
  │       ├─ 凸包 → 多边形顶点 (x, y)
  │       ├─ z 均值 - ground_z → height
  │       └─ 按位置匹配语义名称
  │
  ├─ 投影俯视图 → field_map.png
  │
  └─ 写入 field_geom.yaml
```

### 各区域语义名匹配规则

```python
for cluster in clusters:
    cx, cy = cluster.center
    if cx <  10: name = "Self_Fortress"          # 己方要塞，在场地最左边
    elif cx > 20 and cy > -9: name = "Enemy_Fortress"  # 敌方要塞
    elif cx > 20: name = "Enemy_Buff"             # 敌方增益区
    elif cx > 15: name = "Right_Road"             # 右路
    elif ...     : name = "Left_Road"             # 左路
    else         : name = "Middle_Line"           # 中线
```

---

## 各文件改动

### radar_utils.cpp / .h

- `parser` 构造函数：加载 `field_geom.yaml` 替代硬编码高度
- `parser` 加载 `scene_rules.yaml`：透视参考点、height_threshold、armor_height、fallback_point
- `Parser_Points::ReadPoints`：从 `field_geom.yaml` 读新格式
- `get_2d()` 参考点从 `perspective_ref_points` 读
- `parse()` 的阈值和回退点从 `scene_rules.yaml` 读

### resolve.cpp

- `minimap = cv::imread("config/RM2025.png")` → `"config/field_map.png"`
- `15` / `28` → 从 `field_geom.yaml` 读 `field_size_x/y`

### debug_map.cpp

- `"config/RM2025.png"` → `"config/field_map.png"`
- 所有 `/ 28` `/ 15` 替换为 `field_size_x / field_size_y`
- 所有 `28 - x` `15 - y` 替换为 `field_size_x - x` / `field_size_y - y`
- 英雄判据坐标 → 从 `scene_rules.yaml` 读 `hero_detection`

### dynamic_cloud.cpp

- `"config/RM2025.pcd"` → `"config/field.pcd"`
- `x:3~28, y:0~15, z:0~1.4` → 从 `scene_rules.yaml` 读 `field_bounds`
- 对角线排除区域 → 从 `field_bounds.exclude_regions` 读
- 飞镖/无人机区域 → 从 `scene_rules.yaml` 对应字段读

### localization.cpp

- `"config/RM2025.pcd"` → `"config/field.pcd"`

---

## 换场地流程

```
1. 雷达架在场地角落 2~3m 高，录 15~20 秒点云
      ↓
2. tools/bag_to_pcd.py → config/field.pcd
      ↓
3. scripts/pcd_to_config.py → field_geom.yaml + field_map.png
      ↓
4. 确认 scene_rules.yaml 是否需要调整（同类型场地不用动）
      ↓
5. 代码读配置文件，不需要重新编译
```

## 配置文件变更一览

| 文件 | 换场地就改 | 改一次就行 |
|------|-----------|-----------|
| `field.pcd` | ✅ 重新扫 | |
| `field_geom.yaml` | ✅ 自动生成 | |
| `field_map.png` | ✅ 自动生成 | |
| `scene_rules.yaml` | | ✅ 透视参考点 + 功能区域，同类型场地不用动 |
| `camera_params.yaml` | | ✅ 换相机才改 |
| `out_matrix.yaml` | | ✅ 拆装相机才改 |
