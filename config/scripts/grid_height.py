import cv2
import numpy as np

# 1. 读取 PNG 图像
img = cv2.imread('2_elevation_gray256.png', cv2.IMREAD_GRAYSCALE)
if img is None:
    print("错误：无法读取图像")
    exit(1)

H, W = img.shape
print(f"图像尺寸: {W} x {H}")
print(f"灰度映射: 0=-0.187180m, 254=1.969253m, 255=无数据")

# 高度映射参数
H_MIN = -0.187180
H_MAX = 1.969253
SCALE = (H_MAX - H_MIN) / 254  # 0.008490 m/级

# 2. 将图像划分为 20x20 的格子
GRID_ROWS = 20
GRID_COLS = 20
cell_h = H / GRID_ROWS  # 每格像素高度
cell_w = W / GRID_COLS   # 每格像素宽度

print(f"每格大小: {cell_w:.1f} x {cell_h:.1f} 像素\n")

# 3. 遍历每个格子，计算高度
print(f"{'格子':>6} | {'像素范围':>25} | {'有效像素':>8} | {'最低高度(m)':>12} | {'最高高度(m)':>12} | {'平均高度(m)':>12}")
print("-" * 95)

for row in range(GRID_ROWS):
    for col in range(GRID_COLS):
        # 计算像素范围
        y1 = int(row * cell_h)
        y2 = int((row + 1) * cell_h)
        x1 = int(col * cell_w)
        x2 = int((col + 1) * cell_w)
        
        # 提取该格子内的像素
        cell = img[y1:y2, x1:x2]
        
        # 过滤无数据像素（255）
        valid_mask = cell != 255
        valid_pixels = cell[valid_mask]
        
        if len(valid_pixels) == 0:
            print(f"[{row:2d},{col:2d}] | [{y1:4d}:{y2:4d}, {x1:4d}:{x2:4d}] | {'0':>8} | {'---':>12} | {'---':>12} | {'---':>12}")
        else:
            h_vals = valid_pixels.astype(np.float64) * SCALE + H_MIN
            h_min = h_vals.min()
            h_max = h_vals.max()
            h_avg = h_vals.mean()
            count = len(valid_pixels)
            print(f"[{row:2d},{col:2d}] | [{y1:4d}:{y2:4d}, {x1:4d}:{x2:4d}] | {count:>8} | {h_min:>12.4f} | {h_max:>12.4f} | {h_avg:>12.4f}")

# 4. 输出统计摘要
valid_img = img[img != 255]
if len(valid_img) > 0:
    total_h = valid_img.astype(np.float64) * SCALE + H_MIN
    print(f"\n=== 统计摘要 ===")
    print(f"总有效像素: {len(valid_img)} / {H * W}")
    print(f"整体高度范围: {total_h.min():.4f} ~ {total_h.max():.4f} m")
    print(f"整体平均高度: {total_h.mean():.4f} m")