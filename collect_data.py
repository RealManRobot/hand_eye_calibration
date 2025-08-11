# coding=utf-8
import json
import logging,os

import time
import requests
import sys
import numpy as np
import cv2
import pyrealsense2 as rs

from libs.log_setting import CommonLog
from libs.auxiliary import create_folder_with_date, get_ip, popup_message

cam0_origin_path = create_folder_with_date() # 提前建立好的存储照片文件的目录


logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)

def callback(frame):

    scaling_factor = 2.0
    global count

    cv_img = cv2.resize(frame, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)
    cv2.imshow("Capture_Video", cv_img)  # 窗口显示，显示名为 Capture_Video

    k = cv2.waitKey(30) & 0xFF  # 每帧数据延时 1ms，延时不能为 0，否则读取的结果会是静态帧

    if k == ord('s'):  # 若检测到按键 ‘s’，打印字符串

        state,pose = send_cmd()
        logger_.info(f'获取状态：{"成功" if state else "失败"}，{f"当前位姿为{pose}" if state else None}')
        if state:

            filename = os.path.join(cam0_origin_path,"poses.txt")

            with open(filename, 'a+') as f:
                # 将列表中的元素用空格连接成一行
                pose_ = [str(i) for i in pose]
                new_line = f'{",".join(pose_)}\n'
                # 将新行附加到文件的末尾
                f.write(new_line)

            image_path = os.path.join(cam0_origin_path,f"{str(count)}.jpg")
            cv2.imwrite(image_path , cv_img)
            logger_.info(f"===采集第{count}次数据！")

        count += 1

    else:
        pass


def send_cmd(get_pose=True):
    """
    通过HTTP GET请求从机械臂获取姿态(pose)数据。

    参数:
    get_pose: 如果为True，则获取姿态数据。目前只支持True。

    返回:
    如果get_pose为True, 返回一个元组 (状态, pose数据或错误信息)。
    状态为True表示成功, False表示失败。
    pose数据是一个包含xyzrpy坐标的列表。
    """
    if not get_pose:
        logger_.warning("send_cmd called with get_pose=False, which is not supported in HTTP mode.")
        return (False, "Operation not supported")

    url = "http://192.168.110.91:8080"
    try:
        response = requests.get(url, timeout=5)  # 设置5秒超时
        response.raise_for_status()  # 如果状态码不是2xx，则引发HTTPError

        pose_data = response.json()
        logger_.info(f'Received pose data: {pose_data}')

        if isinstance(pose_data, list) and len(pose_data) == 6:
            return (True, pose_data)
        else:
            logger_.error(f"Invalid pose data format: {pose_data}")
            return (False, f"Invalid pose data format: {pose_data}")

    except requests.exceptions.RequestException as e:
        logger_.error(f"HTTP request failed: {e}")
        return (False, f"HTTP request failed: {e}")
    except json.JSONDecodeError as e:
        logger_.error(f"Failed to decode JSON response: {e}")
        return (False, f"Failed to decode JSON response: {e}")
#
def displayD435():

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        pipeline.start(config)
    except Exception as e:
        logger_.error_(f"相机连接异常：{e}")
        popup_message("提醒", "相机连接异常")

        sys.exit(1)

    global count
    count = 1

    logger_.info(f"开始手眼标定程序，当前程序版号V1.0.0")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            callback(color_image)

    finally:

        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':

    displayD435()
