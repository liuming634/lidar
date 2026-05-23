#!/usr/bin/env python3
"""Wrapper — 调用 config/scripts/ 下的标定脚本"""
import subprocess, sys
subprocess.run([sys.executable, "config/scripts/calibrate_intrinsic.py"] + sys.argv[1:])
