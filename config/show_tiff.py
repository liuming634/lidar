import cv2
import numpy as np
import matplotlib.pyplot as plt

# 读取 TIFF 文件
filename = '1.tiff'
elevation = cv2.imread(filename, cv2.IMREAD_UNCHANGED)

if elevation is None:
    print(f"错误：无法读取文件 '{filename}'")
    exit(1)

print(f"Shape: {elevation.shape}, dtype: {elevation.dtype}")

# 处理 NoData
elevation = elevation.astype(np.float64)
elevation[np.isinf(elevation)] = np.nan
nodata_mask = elevation <= -0.199
elevation[nodata_mask] = np.nan

# 提取有效数据范围
valid = elevation[~np.isnan(elevation)]
evmin, evmax = valid.min(), valid.max()

# 将 NoData 设为白色以便显示
elevation_display = np.copy(elevation)
elevation_display[np.isnan(elevation_display)] = evmax + (evmax - evmin) * 0.01  # 稍高于最大值
# 使用 gray_r 反转：低处亮（白），高处暗（黑）
# 或者用 gray：低处黑，高处白
fig, ax = plt.subplots(1, 1, figsize=(12, 8))
im = ax.imshow(elevation_display, cmap='gray', vmin=evmin, vmax=evmax, interpolation='nearest')
ax.set_facecolor('white')
fig.colorbar(im, ax=ax, label='Elevation (m)')
ax.set_title(f'TIFF Elevation Map ({elevation.shape[1]}x{elevation.shape[0]})')
ax.set_xlabel('Pixel X')
ax.set_ylabel('Pixel Y')
plt.tight_layout()
plt.show()