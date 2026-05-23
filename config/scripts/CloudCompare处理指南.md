
# CloudCompare 生成高程图 完整操作指南与注意事项

## 🎯 目标
利用点云的 Z 轴高程信息，在 CloudCompare 中生成连续的栅格高程图（灰度图像），并导出为包含真实高程值的 GeoTIFF 文件，供 GIS 或编程分析使用。

---

## 📋 操作步骤

### 1. 导入点云并检查
- 拖入或通过 `File` → `Open` 打开点云文件（`.las`、`.ply`、`.txt` 等）。
- 确认点云带有 **Z 坐标**（高度）。
- 若弹出 `Global Shift` 提示，选 **`YES`** 或 **`Auto`** 避免显示精度问题。

### 2. 启动 Rasterize（网格化）
- 左侧 `DB Tree` 中单击选中点云。
- 菜单 `Tools` → `Projection` → **`Rasterize`**。

### 3. 设置核心参数
- **`step`（网格步长）**  
  决定生成图像的分辨率。**值越小像素越精细，但空洞越多**；值越大越连续，细节越少。  
  - 新手可先点 **`Estimate`** 自动推荐，再微调。  
  - 原则：让网格刚好能连续覆盖整个区域，且空洞极少。
- **`direction`（投影方向）**  
  选 **`Z`**，从上向下投影。
- **`cell height`（高度取值方式）**  
  - `average`（平均高度）：最常用，适合平缓地形。  
  - `minimum`（最低点）：生成 DEM（去除树木/建筑后使用）。  
  - `maximum`（最高点）：生成 DSM（含地表物体高度）。

### 4. 更新网格并观察连续性
- 点击 **`Update Grid`** 生成预览。
- 检查视窗中网格是否**紧密拼接成一片连续表面**。  
  - 若呈分散的孤立灰色方块 → **增大 `step` 值**，直至连续。  
  - 若仍有小空洞，可在 `Empty cells` 下拉选 **`Interpolate`** 填充。

### 5. 渲染为灰度显示（可选）
- 关闭或最小化 `Rasterize` 窗口。
- 选中刚生成的网格，菜单 `Edit` → `Scalar fields` → `Export coordinate(s) to SF(s)`（创建 Z 坐标标量场）。
- 在左下角 **`Properties`** 面板（若看不到按 `F8` 或 `Display` → `Properties` 调出）中，找到 `SF display params` 区域的 **`Color Scale`**（彩色条），点击选择 **`Gray`**（灰度）。
- 视图即变为灰度高程图：**越暗越低，越亮越高**。

### 6. 导出 GeoTIFF（保存真实高程值）
- 回到 `Rasterize` 窗口，直接点击 **`Raster`** 按钮（即使颜色变回彩色也没关系）。
- 在弹出的 `Export Raster` 对话框中：  
  - **`File format`** 选择 **`GeoTIFF (*.tif)`**。  
  - 检查 `Projection` 等地理参考（若有）。  
  - 命名并保存。

---

## ⚠️ 常见问题与关键注意事项

| 问题现象 | 原因 | 解决方法 |
|----------|------|----------|
| **生成的是分散的灰色点，不是连续表面** | `step` 太小，点间距大于网格尺寸 | 增大 `step` 并 `Update Grid`，直到网格彼此紧密连接 |
| **GeoTIFF 在 Linux 图片查看器里显示为黑白棋盘格** | 普通看图软件无法正确渲染 32 位浮点型单波段高程数据 | **不代表数据错误**，改用 QGIS 打开或 OpenCV / Python 读取 |
| **`Rasterize` 导出按钮是灰色不可点** | 必须先 `Update Grid` 生成网格才能导出 | 点一次 `Update Grid`，颜色变化不用管，然后点 `Raster` |
| **点击 `Update Grid` 后颜色变回彩虹色** | 软件默认着色被重置，与数据无关 | 直接导出 GeoTIFF，里面的高程值完全正确；如需灰度视图可再次到 Properties 改成 Gray |
| **导出的 GeoTIFF 含有 NoData 空洞** | 点云密度不足或 step 太小导致部分网格无点 | 增大 step 或在 `Empty cells` 选 Interpolate；也可在代码中将 NoData 设为 NaN 或插值处理 |
| **无法找到 Properties 面板** | 界面被折叠或关闭 | 按 `F8` 快捷键，或菜单 `Display` → 勾选 `Properties`，或 `Display` → `Reset GUI layout` |

---

## 🐍 编程读取 GeoTIFF 的高程值（OpenCV / Python）

```python
import cv2
import numpy as np

# 关键：使用 IMREAD_UNCHANGED 保留原始高程值（多为 float32）
elev = cv2.imread('elevation.tif', cv2.IMREAD_UNCHANGED)

# 处理 NoData（假设导出时设为 -9999）
elev[elev == -9999] = np.nan

# 可视化（使用 matplotlib 拉伸显示）
import matplotlib.pyplot as plt
valid = elev[~np.isnan(elev)]
vmin, vmax = np.percentile(valid, [2, 98])
plt.imshow(elev, cmap='gray', vmin=vmin, vmax=vmax)
plt.colorbar(label='Elevation (m)')
plt.show()
```

---

## 📌 流程速查
1. 导入点云 →  
2. `Tools > Projection > Rasterize` →  
3. 设 `step`（点 `Estimate` 后微调）、`direction=Z`、`cell height` →  
4. `Update Grid` → 观察是否连续（不连续就增大 step）→  
5. 导出 `Raster` → `GeoTIFF` →  
6. （可选）在 `Properties` 中改 `Color Scale` 为 `Gray` 观察灰度效果。

如果在任一步骤卡住，对照上述”常见问题”即可快速解决。

---

## ✂️ 附：CloudCompare 裁剪 PCD

在做高程图之前，通常需要先把点云裁剪到场地范围。

### 方法一：用 Crop 工具裁剪

1. 导入点云后，选中该点云
2. 菜单 `Tools` → `Segmentation` → **`Crop`**
3. 在弹出的窗口中勾选要裁剪的维度（x / y / z），填入场地范围：
   ```
   x: 5.0 ~ 30.0
   y: -10.0 ~ 8.0
   z: 0.0 ~ 7.0
   ```
4. 点击 **`Crop`** 确认，框外的点即被删除
5. 左侧 DB Tree 中右键裁剪后的点云 → `Save as` → 选择 **`.pcd`** 格式保存

### 方法二：用 Box 工具裁剪（可视化选择）

1. 导入点云
2. 顶部工具栏点击 **`Box`** 图标（或 `Tools` → `Segmentation` → `Box`）
3. 在视窗中拖出一个立方体框住场地区域
4. 按住 **Shift** 键拖拽框的边或面微调大小
5. 在 DB Tree 中选中裁剪后的点云 → 右键 `Save as` → `.pcd`

### 裁剪后生成高程图

裁剪完的点云范围确定了，再用 `Tools` → `Projection` → `Rasterize` 做高程图，这时生成的 GeoTIFF 像素范围就和场地 `field_width × field_height` 对应上了。
