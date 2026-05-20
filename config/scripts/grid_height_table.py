import cv2
import numpy as np
import csv

# 1. 读取 PNG 图像（只读，不修改）
img = cv2.imread('2_elevation_gray256.png', cv2.IMREAD_GRAYSCALE)
if img is None:
    print("错误：无法读取图像")
    exit(1)

H, W = img.shape

# 高度映射参数
H_MIN = -0.187180
H_MAX = 1.969253
SCALE = (H_MAX - H_MIN) / 254

# 2. 划分 20x20 格子，计算每格平均高度
GRID_ROWS = 20
GRID_COLS = 20
cell_h = H / GRID_ROWS
cell_w = W / GRID_COLS

height_table = np.full((GRID_ROWS, GRID_COLS), np.nan)

for row in range(GRID_ROWS):
    for col in range(GRID_COLS):
        y1 = int(row * cell_h)
        y2 = int((row + 1) * cell_h)
        x1 = int(col * cell_w)
        x2 = int((col + 1) * cell_w)
        cell = img[y1:y2, x1:x2]
        valid = cell[cell != 255]
        if len(valid) > 0:
            height_table[row, col] = valid.astype(np.float64).mean() * SCALE + H_MIN

# 3. 保存为 CSV 文件
csv_file = 'grid_height_20x20.csv'
with open(csv_file, 'w', newline='') as f:
    writer = csv.writer(f)
    # 表头
    header = ['行\\列'] + [f'col{j}' for j in range(GRID_COLS)]
    writer.writerow(header)
    for i in range(GRID_ROWS):
        row_data = [f'row{i}']
        for j in range(GRID_COLS):
            val = height_table[i, j]
            if np.isnan(val):
                row_data.append('无数据')
            else:
                row_data.append(f'{val:.4f}')
        writer.writerow(row_data)

print(f"已保存: {csv_file}")
print(f"尺寸: {GRID_ROWS} x {GRID_COLS}")
print(f"高度范围: {np.nanmin(height_table):.4f} ~ {np.nanmax(height_table):.4f} m")