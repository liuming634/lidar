# 场景适配修改指南

本文档记录适配不同比赛/场景时需要修改的内容。

---

## 整体架构

```
                        resolve（ROS 2 节点层）
                        ├── 接收 2D 检测结果
                        ├── 调用 parser 做坐标转换
                        └── 发布 3D 坐标 + 绘制小地图
                            │
                            ▼
┌──────────────── utils（算法层）─────────────────────┐
│  parser                                              │
│  ├── 读取相机标定参数（camera_params.yaml）           │
│  ├── 读取外参矩阵（out_matrix.yaml）                  │
│  ├── 读取多边形区域定义（RM2025_Points.yaml）         │
│  ├── get_height() → 判断点落在哪个区域，取对应高度     │
│  └── get_2d()     → 透视变换：图像 2D → 世界 3D       │
└──────────────────────────────────────────────────────┘
```

---

## 需要修改的内容

### 1. 配置文件（config/）

| 文件 | 用途 | 说明 |
|------|------|------|
| `config/camera_params.yaml` | 相机内参 | 换相机就要重新标定 |
| `config/out_matrix.yaml` | 相机外参（旋转+平移） | 相机安装位置变了就要重标 |
| `config/RM2025_Points.yaml` | 6 个多边形区域的顶点定义（3D 世界坐标） | 场地不同，区域位置和高度都要改 |

### 2. 代码中的硬编码常量

**`resolve.cpp`**（ROS 节点层）：

- `minimap = cv::imread("config/RM2025.png")` — 小地图图片，换场景要替换
- `send_point.y = 15 - center_point.y` — `15` 是赛场半场宽度（Y方向）
- `center_point.y = 15 + center_point.y` — 同上
- `(Map_clone.cols * center_point.x) / 28` — `28` 是赛场长度（X方向）
- `(Map_clone.rows * (15 - center_point.y) / 15)` — 小地图像素映射

**`radar_utils.cpp`**（算法层）：

- `parser::get_2d()` 中的参考平面四角坐标 `(12, -6) ~ (16, -8)` — 这些是透视变换的参考点，与场地尺寸和相机视野有关
- `parser::parse()` 中的阈值 `0.79` — 高度阈值，超出返回固定点 `(19.322, -1.915)`
- `ARMOR_HEIGHT = 0.15` — 装甲板中心离地高度

**`debug_map.cpp`**（调试可视化）：

- 场地尺寸常量、绘制比例等

### 3. 各区域的对应高度

在 `RM2025_Points.yaml` 中定义，当前 6 个区域：

| 区域 | 当前高度 | 说明 |
|------|----------|------|
| `Middle_Line` | 0.3m | 中场线（路面高出地面） |
| `Left_Road` | 0.2m | 左侧道路 |
| `Right_Road` | 0.2m | 右侧道路 |
| `Enemy_Buff` | 0.6m | 敌方增益区（有高台） |
| `Self_Fortress` | 0.15m | 我方堡垒 |
| `Enemy_Fortress` | 0.15m | 敌方堡垒 |

> 新场景需要根据实际地形重新划分区域和设定高度。

---

## 修改步骤

### Step 1：相机标定
- 内参标定 → 更新 `config/camera_params.yaml`
- 外参标定 → 更新 `config/out_matrix.yaml`

### Step 2：录制静态地图 + 配置 debug_map
- 用录制的俯视图替换 `config/RM2025.png`
- 调整 `debug_map.cpp` 中的绘制参数

### Step 3：定义多边形区域
- 根据新场地图像，在 `RM2025_Points.yaml` 中标注 6 个区域的 3D 顶点坐标
- 根据实际地形测量，设置每个区域的 height

### Step 4：修改代码常量
- `resolve.cpp` 中的场地尺寸（15、28）
- `radar_utils.cpp` 中的参考平面坐标
- 如果区域数量变了，还要改 `parser` 的 `points_map` 初始化

### Step 5：编译测试
- 确认 2D 检测点能正确映射到场地 3D 坐标
- 在小地图上验证绘制位置是否正确





需要作修改的文件：
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/config/out_matrix.yaml
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/config/RM2025_Points.yaml
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/config/RM2025.png
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/config/RM2025.pcd
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/src/fusion/debug_map/debug_map.cpp
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/src/lidar/dynamic_cloud/src/dynamic_cloud.cpp
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/src/tdt_vision/calibrate/src/calibrate.cpp
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/src/tdt_vision/launch
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/src/tdt_vision/maps/map.yaml
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/src/tdt_vision/resolve/src/resolve.cpp
/home/lm/Ubuntu/code/rm/T-DT_Radar_rebuild/src/tdt_vision/utils/src/radar_utils.cpp

