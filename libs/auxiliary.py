import logging
import platform
import subprocess
import os
from datetime import datetime
import tkinter as tk
from tkinter import messagebox
import re

from libs.log_setting import CommonLog

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


def ping(ip):

    # 根据操作系统选择合适的 ping 命令
    if platform.system().lower() == 'windows':
        ping_cmd = f'ping -n 1 {ip}'
    else:
        ping_cmd = ["ping","-c","1",f"{ip}"]


    logger_.info(f'command:{ping_cmd}')


    # 执行命令并获取输出
    response = subprocess.run(ping_cmd, stdout=subprocess.PIPE)

    # 检查响应结果
    if response.returncode == 0:
        return True
    else:
        return False


# 主函数，依次尝试ping两个IP地址
def get_ip():

    ip1 = "192.168.0.18"
    ip2 = "192.168.1.18"

    if ping(ip1):

        print(f"Successfully pinged {ip1}")
        return ip1

    elif ping(ip2):

        print(f"Successfully pinged {ip2}")
        return ip2

    else:

        print("Unable to ping both IP addresses")
        return False


def create_folder_with_date():


    # 获取当前日期，格式为YYYYMMDD
    today = datetime.now().strftime('%Y%m%d')

    prefix_files = "eye_hand_data"


    # 构建基础文件夹名称
    base_folder_name = os.path.join(prefix_files,f"data{today}")

    # 初始化索引和文件夹完整路径
    index = 0
    folder_path = base_folder_name

    # 检查是否需要添加数字后缀来避免重名
    while os.path.exists(folder_path):
        index += 1
        folder_path = f"{base_folder_name}{str(index).zfill(2)}"

    # 创建文件夹


    os.makedirs(folder_path)
    logger_.info(f"create folder {folder_path}")
    return folder_path

def popup_message(title, message):

    # 创建一个新的Tk窗口实例
    root = tk.Tk()
    root.withdraw()  # 隐藏主窗口

    # 设置为顶层窗口
    root.attributes('-topmost', True)

    # 显示消息框
    messagebox.showinfo(title, message)

    # 销毁Tk窗口实例
    root.destroy()


def find_latest_data_folder(path):

    # 正则表达式匹配 "dataYYYYMMDD" 和 "dataYYYYMMDDNN"
    pattern = re.compile(r'^data(\d{8})(\d*)$')

    # 列出路径下所有项并筛选出符合模式的文件夹
    folders = [f for f in os.listdir(path) if os.path.isdir(os.path.join(path, f)) and pattern.match(f)]

    # 如果没有找到符合条件的文件夹，则返回None
    if not folders:
        return None

    print(folders)

    # 使用自定义排序规则：先按日期排序，如果日期相同，则按随后的数字排序（如果存在）
    folders.sort(key=lambda x: (pattern.match(x).group(1), pattern.match(x).group(2)), reverse=True)

    # 返回第一个元素，即日期最大且数字最大（如果有）的文件夹名称
    return folders[0]

if __name__ == "__main__":

    create_folder_with_date()
