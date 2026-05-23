#!/usr/bin/env python3
"""
相机外参标定 (PnP) — 基于已有的 ROS2 标定节点逻辑
从 radar_config.yaml 读取 5 个场地 3D 特征点，
在图像上点击对应位置 → solvePnP → 写入 out_matrix.yaml

用法:
  python3 calibrate_extrinsic.py --image photo.jpg
  python3 calibrate_extrinsic.py --camera 0

操作: 鼠标左键选点 → 按 'n' 确认 → 选完 5 个点自动解算
"""

import argparse
import cv2
import numpy as np
import os
import sys
import yaml

# 切换到脚本所在目录，保证相对路径正确
os.chdir(os.path.dirname(os.path.abspath(__file__)))

YAML_INTRINSIC = "../camera_params.yaml"
YAML_CONFIG    = "../radar_config.yaml"
YAML_EXTRINSIC = "../out_matrix.yaml"

POINT_NAMES = ["self_fortress", "self_tower", "enemy_base", "enemy_tower", "enemy_high"]
POINT_LABELS = ["我方基地", "我方前哨站", "敌方基地", "敌方前哨站", "敌方高地"]


def load_intrinsic(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f"[错误] 无法打开 {path}"); sys.exit(1)
    mtx = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("dist_coeffs").mat()
    fs.release()
    if mtx is None or mtx.size == 0:
        print("[错误] camera_matrix 为空"); sys.exit(1)
    if dist is None or dist.size == 0:
        print("[警告] 未找到畸变系数，使用零畸变")
        dist = np.zeros((1, 5), dtype=np.float64)
    print(f"[OK] 内参加载: fx={mtx[0,0]:.2f}, fy={mtx[1,1]:.2f}, cx={mtx[0,2]:.2f}, cy={mtx[1,2]:.2f}")
    return mtx, dist


def load_calib_points(path):
    if not os.path.exists(path):
        print(f"[错误] 找不到 {path}"); sys.exit(1)
    with open(path) as f:
        lines = [l for l in f if not l.startswith("%")]
        cfg = yaml.safe_load("".join(lines))
    pts_cfg = cfg.get("calibrate_points", {})
    pts_3d, names = [], []
    for key, label in zip(POINT_NAMES, POINT_LABELS):
        p = pts_cfg.get(key)
        if p is None:
            print(f"[错误] 配置中缺少 {key}"); sys.exit(1)
        pts_3d.append(np.float32([p["x"], p["y"], p["z"]]))
        names.append(label)
    print(f"[OK] 加载了 {len(pts_3d)} 个标定参考点:")
    for i, (name, pt) in enumerate(zip(names, pts_3d)):
        print(f"  {i+1}. {name}: ({pt[0]:.2f}, {pt[1]:.2f}, {pt[2]:.2f})")
    return pts_3d, names


def save_extrinsic(rvec, tvec, path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("world_rvec", rvec)
    fs.write("world_tvec", tvec)
    fs.release()
    print(f"[OK] 外参写入 {path}")
    print(f"  world_rvec: {rvec.flatten()}")
    print(f"  world_tvec: {tvec.flatten()}")


def main():
    parser = argparse.ArgumentParser(description="相机外参标定 (PnP)")
    parser.add_argument("--image", help="图片路径")
    parser.add_argument("--camera", type=int, help="本地摄像头编号")
    args = parser.parse_args()

    mtx, dist = load_intrinsic(YAML_INTRINSIC)
    pts_3d, labels = load_calib_points(YAML_CONFIG)

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[错误] 无法读取 {args.image}"); sys.exit(1)
    elif args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"[错误] 无法打开摄像头 {args.camera}"); sys.exit(1)
        ret, img = cap.read()
        cap.release()
        if not ret:
            print("[错误] 无法捕获图像"); sys.exit(1)
    else:
        parser.print_help()
        print("\n示例:")
        print("  python3 calibrate_extrinsic.py --image photo.jpg")
        print("  python3 calibrate_extrinsic.py --camera 0")
        sys.exit(0)

    h, w = img.shape[:2]
    win_w, win_h = (1440, int(1440 * h / w)) if w > h else (int(900 * w / h), 900)
    scale = win_w / w
    disp_img = cv2.resize(img, (win_w, win_h))

    win_name = "Extrinsic Calibration"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, win_w, win_h)
    cv2.namedWindow("ROI", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ROI", 400, 400)
    cv2.moveWindow("ROI", 0, 0)

    pick_pts = []
    current_idx = 0
    temp_x, temp_y = -1, -1
    temp_valid = False
    confirming = False

    def mouse_callback(event, x, y, flags, param):
        nonlocal temp_x, temp_y, temp_valid, confirming
        if event == cv2.EVENT_LBUTTONDOWN and not confirming and current_idx < len(pts_3d):
            temp_x, temp_y = x, y
            temp_valid = True
            confirming = True
            print(f"  点 {current_idx + 1}: 初始位置 ({x}, {y})，可用 WASD 微调，按 'n' 确认")

    cv2.setMouseCallback(win_name, mouse_callback)

    print(f"\n操作: 左键点击选择第 1 个点 ({labels[0]}) → WASD 微调 → 按 'n' 确认")
    print(f"      按 ESC 跳过/退出\n")

    while current_idx < len(pts_3d):
        view = disp_img.copy()

        # ROI 窗口（微调时显示）
        if temp_valid and confirming:
            pad = 50
            x1 = max(0, temp_x - pad)
            y1 = max(0, temp_y - pad)
            x2 = min(win_w, temp_x + pad)
            y2 = min(win_h, temp_y + pad)
            roi = disp_img[y1:y2, x1:x2]
            if roi.size > 0:
                roi_big = cv2.resize(roi, (400, 400))
                cv2.line(roi_big, (200, 0), (200, 400), (0, 0, 255), 1)
                cv2.line(roi_big, (0, 200), (400, 200), (0, 0, 255), 1)
                # 显示坐标偏移
                offset_x = temp_x - (x1 + pad)
                offset_y = temp_y - (y1 + pad)
                cx = 200 + int(offset_x * 400 / (2 * pad))
                cy = 200 + int(offset_y * 400 / (2 * pad))
                cv2.circle(roi_big, (cx, cy), 4, (0, 255, 255), -1)
                cv2.imshow("ROI", roi_big)

        # 绘制已确认的点
        for i, pt in enumerate(pick_pts):
            sx, sy = int(pt[0] * scale), int(pt[1] * scale)
            cv2.circle(view, (sx, sy), 6, (0, 255, 0), -1)
            cv2.putText(view, str(i + 1), (sx + 12, sy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 当前选中的点
        if temp_valid:
            cv2.circle(view, (temp_x, temp_y), 6, (0, 255, 255), -1)
            cv2.line(view, (temp_x - 15, temp_y), (temp_x + 15, temp_y), (0, 255, 255), 1)
            cv2.line(view, (temp_x, temp_y - 15), (temp_x, temp_y + 15), (0, 255, 255), 1)

        prompt = f"点 {current_idx + 1}/{len(pts_3d)}: {labels[current_idx]}"
        cv2.putText(view, prompt, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(view, "左键选点  WASD微调  n=确认  ESC=退出",
                    (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.imshow(win_name, view)

        key = cv2.waitKey(10) & 0xFF
        if key == 27:
            print("用户退出")
            confirming = False
            temp_valid = False
            cv2.destroyWindow("ROI")
            break
        elif key == ord('n') and temp_valid and confirming:
            orig_x = int(temp_x / scale)
            orig_y = int(temp_y / scale)
            pick_pts.append((orig_x, orig_y))
            print(f"  ✓ 点 {current_idx + 1} ({labels[current_idx]}): ({orig_x}, {orig_y})")
            current_idx += 1
            temp_valid = False
            confirming = False
            cv2.destroyWindow("ROI")
            if current_idx < len(pts_3d):
                print(f"左键点击第 {current_idx + 1} 个点 ({labels[current_idx]})")
        elif confirming and temp_valid:
            dx, dy = 0, 0
            if key == ord('w'): dy = -1
            elif key == ord('s'): dy = 1
            elif key == ord('a'): dx = -1
            elif key == ord('d'): dx = 1
            if dx or dy:
                temp_x = max(0, min(win_w - 1, temp_x + dx))
                temp_y = max(0, min(win_h - 1, temp_y + dy))

    cv2.destroyAllWindows()

    if len(pick_pts) < 4:
        print("[错误] 有效点数不足 4 个，无法标定"); sys.exit(1)

    # solvePnP
    print("\n--- 运行 solvePnP ---")
    real_pts = np.array(pts_3d, dtype=np.float64)
    img_pts = np.array(pick_pts, dtype=np.float64).reshape(-1, 1, 2)
    ret, rvec, tvec = cv2.solvePnP(real_pts, img_pts, mtx, dist, flags=cv2.SOLVEPNP_EPNP)

    # 重投影误差
    proj, _ = cv2.projectPoints(real_pts, rvec, tvec, mtx, dist)
    errors = [np.linalg.norm(np.float32(p) - proj[i].flatten()[:2]) for i, p in enumerate(pick_pts)]
    print(f"各点重投影误差: {[f'{e:.2f}' for e in errors]}")
    print(f"平均重投影误差: {np.mean(errors):.2f} px")

    save_extrinsic(rvec, tvec, YAML_EXTRINSIC)


if __name__ == "__main__":
    main()
