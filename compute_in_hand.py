# coding=utf-8
"""眼在手上：计算相机坐标系到机械臂末端坐标系的变换。"""
import argparse
import logging

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from libs.calibration import calibrate


def func(data_path=None):
    return calibrate(data_path, eye_to_hand=False)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    np.set_printoptions(precision=8, suppress=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data_path', nargs='?', help='采集数据目录，默认使用最新一批')
    args = parser.parse_args()
    try:
        rotation, translation = func(args.data_path)
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as exc:
        parser.exit(1, f'标定失败：{exc}\n')
    print('旋转矩阵：\n', rotation)
    print('平移向量（米）：\n', translation)
    print('四元数（x,y,z,w）：\n', Rotation.from_matrix(rotation).as_quat())
