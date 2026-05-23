#!/usr/bin/env python3
"""
相机内参标定工具 — 棋盘格法 (海康 MVS SDK)
用法:
  # 实时拍摄（图片存在内存，不写磁盘）
  python3 calibrate_intrinsic.py --capture

  # 实时拍摄并保存图片到目录
  python3 calibrate_intrinsic.py --capture --save-dir calib_images

  # 从已有图片标定（支持 glob 通配符）
  python3 calibrate_intrinsic.py --images "calib_images/*.jpg"

  # 指定棋盘格参数
  python3 calibrate_intrinsic.py --capture --cols 6 --rows 9 --spacing 25
"""

import argparse
import cv2
import glob
import numpy as np
import os
import sys
from ctypes import *

# 切换到脚本所在目录，保证相对路径正确
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ORIG_CWD = os.getcwd()  # 记住用户运行命令的目录
os.chdir(SCRIPT_DIR)

# ---------- 海康 MVS SDK ----------
MVS_PYTHON_PATH = "/opt/MVS/Samples/64/Python/MvImport"
if os.path.exists(MVS_PYTHON_PATH):
    sys.path.append(MVS_PYTHON_PATH)
else:
    print(f"[错误] 找不到 MVS Python SDK: {MVS_PYTHON_PATH}")
    print("请确认海康 MVS 已安装")
    sys.exit(1)

from MvCameraControl_class import *

# ---------- 棋盘格参数（按实际打印尺寸修改）----------
CHESS_COLS = 6          # 内角点列数（棋盘格宽度方向的内角点数）
CHESS_ROWS = 9          # 内角点行数（棋盘格高度方向的内角点数）
CHESS_SPACING_MM = 27.0  # 棋盘格方格边长 (mm)
PATTERN_TYPE = "chessboard"  # "chessboard" / "asymmetric" / "symmetric"
# --------------------------------------------------

YAML_OUT = "../camera_params.yaml"


def save_yaml(camera_matrix, dist_coeffs, reproj_error, filepath):
    h, w = camera_matrix.shape[:2]
    content = f"""%YAML:1.0
---
camera_matrix: !!opencv-matrix
   rows: {h}
   cols: {w}
   dt: d
   data: [{', '.join(f'{v:.6f}' for row in camera_matrix for v in row)}]
dist_coeffs: !!opencv-matrix
   rows: {dist_coeffs.shape[0]}
   cols: {dist_coeffs.shape[1]}
   dt: d
   data: [{', '.join(f'{v:.6f}' for v in dist_coeffs.flatten())}]
reprojection_error: {reproj_error:.6f}
image_size: [{camera_matrix[0,2]*2:.0f}, {camera_matrix[1,2]*2:.0f}]
"""
    with open(filepath, "w") as f:
        f.write(content)
    print(f"[OK] 结果写入 {filepath}")


def find_pattern(img, pattern_type, show=False):
    """检测标定板并返回角点/圆心坐标"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    rows, cols = CHESS_ROWS, CHESS_COLS
    pattern_size = (cols, rows)

    if pattern_type == "chessboard":
        # 棋盘格检测
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            points = corners
        else:
            points = None
    else:
        # 圆点检测（asymmetric / symmetric）
        flags_base = (cv2.CALIB_CB_ASYMMETRIC_GRID if pattern_type == "asymmetric"
                      else cv2.CALIB_CB_SYMMETRIC_GRID)

        # 多策略重试
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)

        ret, points = False, None
        for proc_img, extra_flags in [
            (gray, 0),
            (gray, cv2.CALIB_CB_CLUSTERING),
            (enhanced, cv2.CALIB_CB_CLUSTERING),
        ]:
            ret, points = cv2.findCirclesGrid(proc_img, pattern_size, None, flags_base | extra_flags)
            if ret:
                break

    if ret and show:
        vis = img.copy()
        cv2.drawChessboardCorners(vis, pattern_size, points, ret)
        cv2.imshow("Pattern", vis)
        cv2.waitKey(200)

    return ret, points


def make_object_points(pattern_type):
    """生成标定板 3D 世界坐标"""
    rows, cols = CHESS_ROWS, CHESS_COLS
    spacing = CHESS_SPACING_MM

    if pattern_type == "asymmetric":
        objp = []
        for i in range(rows):
            for j in range(cols):
                objp.append([(2 * j + i % 2) * spacing, i * spacing, 0.0])
        return np.array(objp, dtype=np.float32)
    else:
        # chessboard 和 symmetric circle 都是规则网格
        objp = np.zeros((rows * cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * spacing
        return objp


def calibrate(images, pattern_type):
    """核心标定：传入图像列表（numpy 数组），输出相机矩阵"""
    objp = make_object_points(pattern_type)
    obj_points = []
    img_points = []
    image_sizes = []

    label = "棋盘格" if pattern_type == "chessboard" else "圆点"

    for img in images:
        ret, pts = find_pattern(img, pattern_type, show=True)
        if ret:
            obj_points.append(objp)
            img_points.append(pts)
            image_sizes.append((img.shape[1], img.shape[0]))
        else:
            print(f"  [跳过] 未检测到{label}")

    cv2.destroyAllWindows()

    if len(obj_points) < 5:
        print(f"[错误] 有效图片太少（需要 ≥5 张），标定失败")
        return None

    print(f"\n有效图片: {len(obj_points)} 张，标定中...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_sizes[0], None, None
    )

    # 重投影误差
    total_error = 0
    for i in range(len(obj_points)):
        imgpts, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(img_points[i], imgpts, cv2.NORM_L2) / len(imgpts)
        total_error += error
    reproj_error = total_error / len(obj_points)

    print(f"  重投影误差: {reproj_error:.4f} px")
    print(f"  相机矩阵:\n{mtx}")
    print(f"  畸变系数:\n{dist}")
    return mtx, dist, reproj_error


def grab_mvs_frame(cam, stOutFrame):
    """从海康相机获取一帧并转为 BGR numpy array"""
    ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
    if ret != 0 or stOutFrame.pBufAddr is None:
        return None

    nWidth = stOutFrame.stFrameInfo.nWidth
    nHeight = stOutFrame.stFrameInfo.nHeight
    convert_bufflen = nWidth * nHeight * 3
    DstBuffer = (c_ubyte * convert_bufflen)()

    stConvertParam = MV_CC_PIXEL_CONVERT_PARAM_EX()
    memset(byref(stConvertParam), 0, sizeof(stConvertParam))
    stConvertParam.pSrcData = stOutFrame.pBufAddr
    stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLen
    stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType
    stConvertParam.nWidth = nWidth
    stConvertParam.nHeight = nHeight
    stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed
    stConvertParam.pDstBuffer = DstBuffer
    stConvertParam.nDstBufferSize = convert_bufflen

    ret = cam.MV_CC_ConvertPixelTypeEx(stConvertParam)
    cam.MV_CC_FreeImageBuffer(stOutFrame)

    if ret != 0:
        return None

    rgb = np.frombuffer(DstBuffer, dtype=np.ubyte, count=convert_bufflen).reshape(nHeight, nWidth, 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def init_mvs_camera():
    """初始化海康相机，返回 (cam, stOutFrame)"""
    MvCamera.MV_CC_Initialize()

    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_USB_DEVICE | MV_GIGE_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0:
        print("[错误] 枚举设备失败"); sys.exit(1)
    if deviceList.nDeviceNum == 0:
        print("[错误] 未找到海康相机"); sys.exit(1)

    print(f"[OK] 找到 {deviceList.nDeviceNum} 个海康相机，连接第一个...")
    stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents

    cam = MvCamera()
    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print(f"[错误] 创建句柄失败: {ret}"); sys.exit(1)

    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print(f"[错误] 打开设备失败: {ret}"); sys.exit(1)

    cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_BGR8_Packed)

    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print(f"[错误] 开始采集失败: {ret}"); sys.exit(1)

    stOutFrame = MV_FRAME_OUT()
    memset(byref(stOutFrame), 0, sizeof(stOutFrame))

    print("[OK] 海康相机初始化完成\n")
    return cam, stOutFrame


def close_mvs_camera(cam):
    """关闭海康相机"""
    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    MvCamera.MV_CC_Finalize()


def mode_capture(save_dir):
    """实时拍摄 → 内存中保存 → 直接标定"""
    cam, stOutFrame = init_mvs_camera()

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    images = []
    label = "棋盘格" if PATTERN_TYPE == "chessboard" else "圆点"
    print(f"按 SPACE 拍照（已拍: 0 张）  按 ESC 结束并标定")
    print(f"建议拍摄 ≥15 张{label}在不同角度和距离的照片\n")

    while True:
        frame = grab_mvs_frame(cam, stOutFrame)
        if frame is None:
            continue

        vis = frame.copy()
        text = f"Captured: {len(images)}  SPACE:拍照  ESC:标定"
        cv2.putText(vis, text, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Calibration", vis)
        key = cv2.waitKey(10) & 0xFF
        if key == 27:
            break
        elif key == 32:
            images.append(frame.copy())
            if save_dir:
                path = os.path.join(save_dir, f"calib_{len(images):03d}.png")
                cv2.imwrite(path, frame)
            print(f"  [拍摄] 第 {len(images)} 张")

    close_mvs_camera(cam)
    cv2.destroyAllWindows()

    if len(images) < 5:
        print("[错误] 照片太少，标定失败")
        sys.exit(1)

    print(f"\n共拍摄 {len(images)} 张，开始标定...")
    result = calibrate(images, PATTERN_TYPE)
    if result:
        save_yaml(*result, YAML_OUT)


def mode_images(image_paths):
    """从文件读取图片 → 标定（支持 glob 通配符）"""
    # 展开 glob 通配符 — 先在当前目录（脚本目录）找，再在用户运行目录找
    expanded = []
    for p in image_paths:
        matched = glob.glob(p)
        if not matched:
            matched = glob.glob(os.path.join(ORIG_CWD, p))
        if matched:
            expanded.extend(matched)
        else:
            expanded.append(p)  # 传 literal 给 imread，它自己报错

    images = []
    for path in expanded:
        img = cv2.imread(path)
        if img is not None:
            images.append(img)
            print(f"  [读取] {os.path.basename(path)}")
        else:
            print(f"  [跳过] 无法读取: {path}")

    if len(images) < 5:
        print("[错误] 有效图片太少，标定失败")
        sys.exit(1)

    print(f"\n共读取 {len(images)} 张，开始标定...")
    result = calibrate(images, PATTERN_TYPE)
    if result:
        save_yaml(*result, YAML_OUT)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="相机内参标定 — 棋盘格/圆点")
    parser.add_argument("--capture", action="store_true", help="实时拍摄模式")
    parser.add_argument("--images", nargs="+", help="已有图片路径")
    parser.add_argument("--save-dir", help="拍摄时同时保存图片到目录（可选）")
    parser.add_argument("--rows", type=int, default=CHESS_ROWS, help="内角点行数 / 圆点行数")
    parser.add_argument("--cols", type=int, default=CHESS_COLS, help="内角点列数 / 圆点列数")
    parser.add_argument("--spacing", type=float, default=CHESS_SPACING_MM, help="方格边长 / 圆心间距 (mm)")
    parser.add_argument("--pattern", choices=["chessboard", "asymmetric", "symmetric"],
                        default=PATTERN_TYPE, help="标定板类型")
    parser.add_argument("--output", default=YAML_OUT, help="输出 YAML 路径")
    args = parser.parse_args()

    CHESS_COLS = args.cols
    CHESS_ROWS = args.rows
    CHESS_SPACING_MM = args.spacing
    PATTERN_TYPE = args.pattern
    YAML_OUT = args.output

    if args.capture:
        mode_capture(args.save_dir)
    elif args.images:
        mode_images(args.images)
    else:
        parser.print_help()
        print("\n示例:")
        print("  实时拍摄（棋盘格）:  python3 calibrate_intrinsic.py --capture")
        print("  实时拍摄 + 保存:     python3 calibrate_intrinsic.py --capture --save-dir calib_images")
        print("  读取已有图片:        python3 calibrate_intrinsic.py --images ./calib_*.png")
        print("  圆点标定:            python3 calibrate_intrinsic.py --capture --pattern asymmetric --cols 8 --rows 11")
