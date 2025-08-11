# coding=utf-8

import numpy as np

def compute_target_in_hand_frame(base_to_hand_pose, camera_to_target_pose, eye_on_hand_calib_matrix):
    """
    计算目标物体在机械手坐标系中的位姿。

    此函数适用于“眼在手上”(eye-on-hand)的场景，即相机安装在机器人末端执行器上。
    它利用机器人基座到手部的位姿、相机到目标的位姿以及手眼标定矩阵，
    计算出目标相对于手部的最终位姿。

    数学公式: T_hand_target = T_hand_camera * T_camera_target
    其中 T_hand_camera 是手眼标定矩阵。

    Args:
        base_to_hand_pose (np.ndarray): 机器人基座到末端执行器（手）的4x4位姿变换矩阵。
                                        通常由机器人控制器提供。
        camera_to_target_pose (np.ndarray): 相机坐标系到目标物体的4x4位姿变换矩阵。
                                            通常由视觉系统（相机）检测得出。
        eye_on_hand_calib_matrix (np.ndarray): 手（末端执行器）到相机的4x4位姿变换矩阵。
                                               这是通过手眼标定过程计算得出的“手眼矩阵”。

    Returns:
        np.ndarray: 目标物体在机械手坐标系中的4x4位姿变换矩阵。
    """
    # 验证输入是否为4x4的numpy数组
    assert base_to_hand_pose.shape == (4, 4), "base_to_hand_pose 必须是 4x4 矩阵"
    assert camera_to_target_pose.shape == (4, 4), "camera_to_target_pose 必须是 4x4 矩阵"
    assert eye_on_hand_calib_matrix.shape == (4, 4), "eye_on_hand_calib_matrix 必须是 4x4 矩阵"

    # T_hand_target = T_hand_camera * T_camera_target
    # T_hand_camera 就是手眼标定矩阵 eye_on_hand_calib_matrix
    target_in_hand_frame = np.dot(eye_on_hand_calib_matrix, camera_to_target_pose)

    return target_in_hand_frame

# --- 主程序入口 ---
if __name__ == '__main__':
    # 这是一个示例，展示如何使用上述函数。
    # 在实际应用中，这些矩阵将从机器人和相机实时获取。

    print("--- 手眼标定计算示例 ---")

    # 1. 定义机器人基座到手部的位姿矩阵 (T_base_hand)
    #    假设手部相对于基座平移了 (x=0.5, y=0.2, z=0.4) 米，并且绕Z轴旋转了90度。
    base_to_hand_pose = np.array([
        [0, -1, 0, 0.5],
        [1,  0, 0, 0.2],
        [0,  0, 1, 0.4],
        [0,  0, 0, 1.0]
    ])
    print("\n机器人基座 -> 手部 的位姿 (T_base_hand):\n", base_to_hand_pose)

    # 2. 定义手到相机的标定矩阵 (T_hand_camera)
    #    这是通过手眼标定得到的关键结果。
    #    假设相机相对于手部平移了 (x=0.1, y=0, z=0.05) 米，并且绕Y轴旋转了-90度。
    eye_on_hand_calib_matrix = np.array([
        [ 0, 0, 1, 0.1],
        [ 0, 1, 0, 0.0],
        [-1, 0, 0, 0.05],
        [ 0, 0, 0, 1.0]
    ])
    print("\n手 -> 相机 的标定矩阵 (T_hand_camera):\n", eye_on_hand_calib_matrix)

    # 3. 定义相机检测到的目标位姿 (T_camera_target)
    #    假设相机检测到目标在其前方0.3米处。
    camera_to_target_pose = np.array([
        [1, 0, 0, 0.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.3],
        [0, 0, 0, 1.0]
    ])
    print("\n相机 -> 目标 的位姿 (T_camera_target):\n", camera_to_target_pose)

    # 4. 调用函数计算目标在手部坐标系中的位姿
    target_in_hand_pose = compute_target_in_hand_frame(
        base_to_hand_pose,
        camera_to_target_pose,
        eye_on_hand_calib_matrix
    )

    # 5. 打印最终结果
    print("\n计算结果：目标 -> 手部 的位姿 (T_hand_target):\n", target_in_hand_pose)

    # 从结果矩阵中提取平移向量 (x, y, z)
    translation = target_in_hand_pose[:3, 3]
    print(f"\n目标在手部坐标系下的位置 (x, y, z): ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})")


"""
眼在手外 用采集到的图片信息和机械臂位姿信息计算 相机坐标系相对于机械臂基坐标系的 旋转矩阵和平移向量

"""

import os
import logging

import yaml
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from libs.auxiliary import find_latest_data_folder
from libs.log_setting import CommonLog

from save_poses2 import poses2_main

np.set_printoptions(precision=8,suppress=True)

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


current_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"eye_hand_data")

images_path = os.path.join("eye_hand_data",find_latest_data_folder(current_path))

file_path = os.path.join(images_path,"poses.txt")  #采集标定板图片时对应的机械臂末端的位姿 从 第一行到最后一行 需要和采集的标定板的图片顺序进行对应


with open("config.yaml", 'r', encoding='utf-8') as file:
    data = yaml.safe_load(file)

XX = data.get("checkerboard_args").get("XX") #标定板的中长度对应的角点的个数
YY = data.get("checkerboard_args").get("YY") #标定板的中宽度对应的角点的个数
L = data.get("checkerboard_args").get("L")   #标定板一格的长度  单位为米

def func():

    path = os.path.dirname(__file__)
    print(path)

    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 获取标定板角点的位置
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)     # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
    objp = L*objp

    obj_points = []     # 存储3D点
    img_points = []     # 存储2D点

    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]

    for i in range(1, len(images_num) + 1):   #标定好的图片在images_path路径下，从0.jpg到x.jpg

        image_file = os.path.join(images_path,f"{i}.jpg")

        if os.path.exists(image_file):

            logger_.info(f'读 {image_file}')

            img = cv2.imread(image_file)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

            if ret:

                obj_points.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
                if [corners2]:
                    img_points.append(corners2)
                else:
                    img_points.append(corners)

    N = len(img_points)

    # 标定,得到图案在相机坐标系下的位姿
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

    # logger_.info(f"内参矩阵:\n:{mtx}" ) # 内参数矩阵
    # logger_.info(f"畸变系数:\n:{dist}")  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)

    print("-----------------------------------------------------")

    poses2_main(file_path)
    # 机器人末端在基座标系下的位姿

    csv_file = os.path.join(path,"RobotToolPose.csv")
    tool_pose = np.loadtxt(csv_file,delimiter=',')

    R_tool = []
    t_tool = []

    for i in range(int(N)):

        R_tool.append(tool_pose[0:3,4*i:4*i+3])
        t_tool.append(tool_pose[0:3,4*i+3])

    R, t = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)

    return R,t

if __name__ == '__main__':

    # 旋转矩阵
    rotation_matrix, translation_vector = func()

    # 将旋转矩阵转换为四元数
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()
    x, y, z = translation_vector.flatten()

    logger_.info(f"旋转矩阵是:\n {            rotation_matrix}")

    logger_.info(f"平移向量是:\n {            translation_vector}")

    logger_.info(f"四元数是：\n {             quaternion}")

