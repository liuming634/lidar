# PCD 场景配置框架

用雷达扫描的 PCD 静态地图自动生成场地配置，替代手动测量和硬编码。

---

## 整体数据流

```
                     PCD 处理脚本
  ┌────────────┐    ┌──────────────────────────┐
  │ field.pcd  │───→│ scripts/pcd_to_config.py │
  └────────────┘    │                          │
                    │  地面 → 场地尺寸          │──→ field_size_x/y
                    │  非地面 → DBSCAN 聚类     │──→ 区域多边形 (x,y)
                    │   每个 cluster:           │──→ height (z 均值)
                    │  俯视图渲染               │──→ field_map.png
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

## 配置文件

### field_geom.yaml（自动生成）

替换现有的 `RM2025_Points.yaml`，包含场地几何数据：

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

- 每个点只有 (x, y)，高度统一用 `height` 字段，避免冗余
- OpenCV FileStorage 格式，和现有读取方式兼容

### scene_rules.yaml（手动配置，一次配好）

场景的功能规则，与场地几何无关的人为定义：

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

resolve 管线的流程不变，数据来源从硬编码改成配置文件：

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
| 区域多边形 | `RM2025_Points.yaml` 手填 (x,y,z) | `field_geom.yaml` (x,y) + 独立 height |
| 区域高度 | `radar_utils.cpp` 硬编码 | `field_geom.yaml` 每个 region 的 height |
| 透视参考点 | `radar_utils.cpp` 硬编码 (12,-6)~(16,-8) | `scene_rules.yaml` 读 |
| 高度阈值 / 回退点 | 0.79, (19.322,-1.915) 硬编码 | `scene_rules.yaml` 读 |

---

## PCD 处理脚本

`scripts/pcd_to_config.py`，输入 `config/field.pcd`，输出 `field_geom.yaml` + `field_map.png`：

```
PCD 点云
  │
  ├─ RANSAC 地面拟合 → 地面点
  │    └─ X/Y 边界 → field_size_x, field_size_y
  │    └─ 最高 z  → field_bounds.z_max
  │
  ├─ 非地面点 → DBSCAN 聚类
  │    └─ 每个 cluster:
  │       ├─ 凸包 → 多边形顶点 (x, y)
  │       ├─ z 均值 → height
  │       └─ 按位置匹配语义名（Left_Road / Enemy_Buff…）
  │
  ├─ 投影俯视图 → field_map.png（按高度着色）
  │
  └─ 写入 field_geom.yaml
```

---

## 各文件改动

### radar_utils.cpp / .h

- `parser` 构造函数：加载 `field_geom.yaml` 替代硬编码高度
- `parser` 加载 `scene_rules.yaml`：透视参考点、height_threshold、armor_height、fallback_point
- `Parser_Points::ReadPoints`：从 `field_geom.yaml` 读新格式（点只有 x/y，height 独立）
- `get_2d()` 参考点从 `perspective_ref_points` 读
- `parse()` 的阈值和回退点从 `scene_rules.yaml` 读

### resolve.cpp

- `minimap = cv::imread("config/RM2025.png")` → `"config/field_map.png"`
- `15` / `28` → 从 `field_geom.yaml` 读 `field_size_x/y`

### debug_map.cpp

- `"config/RM2025.png"` → `"config/field_map.png"`
- `#define FIELD_SIZE_X 28`, `#define FIELD_SIZE_Y 15` → 从配置文件读
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
1. 雷达扫场地 → 得到 field.pcd
2. 运行 pcd_to_config.py → 生成 field_geom.yaml + field_map.png
3. 确认 scene_rules.yaml 是否需要调整（同类型场地通常不用动）
4. 所有代码读配置文件，不需要重新编译
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
