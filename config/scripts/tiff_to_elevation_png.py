#!/usr/bin/env python3
"""
TIFF 高程图转 PNG 灰度图脚本

功能：将 16bit TIFF 高程图转换为 256 灰度 PNG 图像
灰度越深 = 高度越高（符合视觉习惯）

用法：
    python tiff_to_elevation_png.py <输入tiff> <输出png>
    例如：python tiff_to_elevation_png.py config/elevation/3.tiff config/elevation/3_elevation_gray256.png
"""

import sys
import cv2
import numpy as np
import tifffile


def tiff_to_gray_png(tiff_path: str, output_path: str) -> dict:
    """
    将 TIFF 高程图转换为 256 灰度 PNG

    Args:
        tiff_path: 输入 TIFF 文件路径
        output_path: 输出 PNG 文件路径

    Returns:
        dict: 包含 z_min, z_max, scale 等参数
    """
    # 读取 TIFF
    data = tifffile.imread(tiff_path)
    print(f"原始形状: {data.shape}")
    print(f"数据类型: {data.dtype}")

    # 获取有效数据范围（排除 nan）
    valid_mask = ~np.isnan(data)
    valid_data = data[valid_mask]

    if len(valid_data) == 0:
        raise ValueError("TIFF 文件中没有有效数据（全是 nan）")

    z_min = float(valid_data.min())
    z_max = float(valid_data.max())

    print(f"有效点数: {len(valid_data)} / {data.size}")
    print(f"高度范围: {z_min:.6f} ~ {z_max:.6f}")

    # 创建 256 灰度图
    # gray = 0  -> 最高点 (z_max)
    # gray = 254 -> 最低点 (z_min)
    # gray = 255 -> 无数据
    gray_img = np.full((data.shape[0], data.shape[1]), 255, dtype=np.uint8)

    # 映射公式: gray = (z_max - height) / (z_max - z_min) * 254
    gray_values = ((z_max - valid_data) / (z_max - z_min) * 254).astype(np.uint8)
    gray_values = np.clip(gray_values, 0, 254)
    gray_img[valid_mask] = gray_values

    # 保存 PNG
    cv2.imwrite(output_path, gray_img)
    print(f"已保存: {output_path}")

    # 计算缩放因子
    scale = (z_max - z_min) / 254

    return {
        "z_min": z_min,
        "z_max": z_max,
        "scale": scale,
        "formula": f"height = gray * {scale:.6f} + ({z_min:.6f})"
    }


def main():
    if len(sys.argv) != 3:
        print("用法: python tiff_to_elevation_png.py <输入tiff> <输出png>")
        print("示例: python tiff_to_elevation_png.py config/elevation/3.tiff config/elevation/3_elevation_gray256.png")
        sys.exit(1)

    tiff_path = sys.argv[1]
    output_path = sys.argv[2]

    result = tiff_to_gray_png(tiff_path, output_path)

    print("\n=== 高度计算公式 ===")
    print(f"z_min = {result['z_min']}")
    print(f"z_max = {result['z_max']}")
    print(f"scale = (z_max - z_min) / 254 = {result['scale']:.6f}")
    print(f"公式: {result['formula']}")

    print("\n=== YAML 配置 ===")
    print(f"image: \"{output_path}\"")
    print(f"z_min: {result['z_min']}")
    print(f"z_max: {result['z_max']}")
    print(f"no_data: 255")


if __name__ == "__main__":
    main()
