import cv2
import numpy as np

# 1. 读取 2.tiff
filename = '2.tiff'
elevation = cv2.imread(filename, cv2.IMREAD_UNCHANGED)

if elevation is None:
    print(f"错误：无法读取文件 '{filename}'")
    exit(1)

# 转灰度（如果是多通道）
if len(elevation.shape) == 3:
    elevation = cv2.cvtColor(elevation, cv2.COLOR_BGR2GRAY)

# 处理 NaN / inf
elevation = elevation.astype(np.float64)
elevation[np.isinf(elevation)] = np.nan
nodata_mask = np.isnan(elevation) | (elevation <= -0.199)
elevation[nodata_mask] = np.nan

# 获取有效高度范围
valid = elevation[~np.isnan(elevation)]
h_min = valid.min()
h_max = valid.max()
print(f"=== 2.tiff 高度数据 ===")
print(f"高度范围: {h_min:.6f} ~ {h_max:.6f} m")
print(f"有效像素: {len(valid)} / {elevation.size}")

# 2. 将高度映射到 0-255 灰度值（256级）
# 0=最低高度(黑), 255=最高高度(白), 255同时用于无数据(白色背景)
gray = np.full(elevation.shape, 255, dtype=np.uint8)  # 背景白色
valid_mask = ~nodata_mask
gray[valid_mask] = np.clip(
    (elevation[valid_mask] - h_min) / (h_max - h_min) * 254,
    0, 254
).astype(np.uint8)

# 3. 保存为 PNG
output = '2_elevation_gray256.png'
cv2.imwrite(output, gray)

step = (h_max - h_min) / 254
print(f"\n已保存: {output}")
print(f"灰度级数: 256 (0~254为高度, 255为无数据)")
print(f"每级对应高度: {step*1000:.4f} mm")
print(f"灰度0 = {h_min:.6f} m")
print(f"灰度254 = {h_max:.6f} m")
print(f"灰度255 = 无数据(白色)")