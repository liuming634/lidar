#!/usr/bin/env python3
"""
高程图高度查询工具

功能：读取 256 灰度高程图，计算每个格子的高度统计信息

用法：
    python grid_height.py [图像路径]
    默认读取: config/elevation/3_elevation_gray256.png
"""

import cv2
import numpy as np
import sys

# 默认图像路径
DEFAULT_IMAGE = "config/elevation/3_elevation_gray256.png"

# 高度映射参数 (3_elevation_gray256.png)
H_MIN = -1.265726
H_MAX = 2.614000
SCALE = (H_MAX - H_MIN) / 254


def load_elevation_image(img_path: str = None):
    """加载高程图"""
    if img_path is None:
        img_path = DEFAULT_IMAGE

    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"错误：无法读取图像 {img_path}")
        exit(1)

    print(f"图像路径: {img_path}")
    print(f"图像尺寸: {img.shape[1]} x {img.shape[0]}")
    print(f"灰度映射: 0(最深/最高) -> {H_MAX}m, 254(最浅/最低) -> {H_MIN}m, 255(无数据)")
    print(f"高度范围: {H_MIN}m ~ {H_MAX}m")
    print(f"SCALE = (H_MAX - H_MIN) / 254 = {SCALE:.6f} m/级")
    print(f"公式: height = gray * {SCALE:.6f} + ({H_MIN:.6f})\n")

    return img


def analyze_grid(img: np.ndarray, grid_rows: int = 20, grid_cols: int = 20):
    """分析网格内的高度统计"""
    H, W = img.shape
    cell_h = H / grid_rows
    cell_w = W / grid_cols

    print(f"网格划分: {grid_rows} x {grid_cols}")
    print(f"每格大小: {cell_w:.1f} x {cell_h:.1f} 像素\n")

    print(f"{'格子':>6} | {'像素范围':>25} | {'有效像素':>8} | {'最低高度(m)':>12} | {'最高高度(m)':>12} | {'平均高度(m)':>12}")
    print("-" * 95)

    for row in range(grid_rows):
        for col in range(grid_cols):
            y1 = int(row * cell_h)
            y2 = int((row + 1) * cell_h)
            x1 = int(col * cell_w)
            x2 = int((col + 1) * cell_w)

            cell = img[y1:y2, x1:x2]
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


def print_summary(img: np.ndarray):
    """输出统计摘要"""
    valid_img = img[img != 255]
    if len(valid_img) > 0:
        total_h = valid_img.astype(np.float64) * SCALE + H_MIN
        print(f"\n=== 统计摘要 ===")
        print(f"总有效像素: {len(valid_img)} / {img.shape[0] * img.shape[1]}")
        print(f"整体高度范围: {total_h.min():.4f} ~ {total_h.max():.4f} m")
        print(f"整体平均高度: {total_h.mean():.4f} m")


def main():
    img_path = sys.argv[1] if len(sys.argv) > 1 else None
    img = load_elevation_image(img_path)
    analyze_grid(img)
    print_summary(img)


if __name__ == "__main__":
    main()
