# coding=utf-8
import json
import logging
import os
from pathlib import Path
import socket
import time

import cv2
import numpy as np

from libs.log_setting import CommonLog
from libs.auxiliary import create_folder_with_date, get_ip

logger_ = CommonLog(logging.getLogger(__name__))
cam0_origin_path = None
count = 1


def save_sample(directory, index, frame, pose):
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError('机械臂位姿必须包含六个有限数值')
    image_path = Path(directory) / f'{index}.jpg'
    if image_path.exists():
        raise ValueError(f'图片已存在，拒绝覆盖：{image_path}')
    try:
        if not cv2.imwrite(str(image_path), frame):
            raise OSError(f'图片保存失败：{image_path}')
        with (Path(directory) / 'poses.txt').open('a+', encoding='utf-8') as stream:
            stream.seek(0, os.SEEK_END)
            offset = stream.tell()
            try:
                stream.write(','.join(str(value) for value in pose) + '\n')
                stream.flush()
                os.fsync(stream.fileno())
            except BaseException:
                stream.seek(offset)
                stream.truncate()
                raise
    except BaseException:
        image_path.unlink(missing_ok=True)
        raise


def callback(frame):
    global count
    preview = cv2.resize(frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_AREA)
    cv2.imshow('Capture_Video', preview)
    key = cv2.waitKey(30) & 0xFF
    if key in (ord('q'), 27):
        return False
    if key == ord('s'):
        state, pose = send_cmd(client, '{"command":"get_current_arm_state"}')
        if not state:
            logger_.error_(f'采集失败，未保存本组数据：{pose}')
            return True
        try:
            save_sample(cam0_origin_path, count, frame, pose)
        except (OSError, ValueError, cv2.error) as exc:
            logger_.error_(f'采集保存失败：{exc}')
            return True
        logger_.info(f'采集第 {count} 组数据成功')
        count += 1
    return True


def send_cmd(client, cmd, get_pose=True):
    """发送 CRLF 结尾的 JSON，限时接收完整且匹配的回包。"""
    try:
        command = json.loads(cmd)['command']
        client.settimeout(3.0)
        client.sendall((cmd.rstrip() + '\r\n').encode('utf-8'))
        decoder = json.JSONDecoder()
        deadline = time.monotonic() + 3.0
        buffer = b''
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('等待匹配回包超时')
            buffer = buffer.lstrip()
            text = ''
            try:
                text = buffer.decode('utf-8')
            except UnicodeDecodeError as exc:
                if exc.reason != 'unexpected end of data':
                    raise ValueError('回包不是有效 UTF-8') from exc
            data = None
            if text:
                try:
                    data, end = decoder.raw_decode(text)
                except json.JSONDecodeError:
                    if b'\n' in buffer:
                        raise ValueError('收到不完整或无效的 JSON 行')
                else:
                    buffer = text[end:].encode('utf-8')
                    if not isinstance(data, dict):
                        raise ValueError('回包必须是 JSON 对象')
                    if data.get('command') == 'error':
                        raise ValueError(f'控制器返回错误：{data.get("msg", data)}')
                    if get_pose and data.get('state') == 'current_arm_state':
                        state = data['arm_state']
                        errors = state['err']
                        if not (type(errors) is int and errors == 0 or isinstance(errors, list)
                                and errors and all(type(e) is int and e == 0 for e in errors)):
                            return False, f'机械臂报错：{errors}'
                        raw = state['pose']
                        if (not isinstance(raw, list) or len(raw) != 6
                                or any(type(v) not in (int, float) for v in raw)
                                or not np.isfinite(raw).all()):
                            raise ValueError('回包 pose 必须包含六个有限数值')
                        return True, [v / (1000000 if i < 3 else 1000) for i, v in enumerate(raw)]
                    if not get_pose and data.get('command') == command:
                        return data.get('change_work_frame', data.get('set_state')) is True
                    continue
            client.settimeout(remaining)
            chunk = client.recv(4096)
            if not chunk:
                raise ConnectionError('控制器连接已关闭')
            if len(buffer) + len(chunk) > 1024 * 1024:
                raise ValueError('控制器回包过长')
            buffer += chunk
    except (OSError, ValueError, KeyError, TypeError) as exc:
        # 超时后的迟到回包无法与下次请求区分，关闭连接以免采到旧位姿。
        client.close()
        logger_.error_(f'机械臂通信失败，请检查连接并重新启动采集：{exc}')
        return (False, str(exc)) if get_pose else False


def displayD435():
    import pyrealsense2 as rs

    global cam0_origin_path, count
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    try:
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        cam0_origin_path = create_folder_with_date()
        camera = {
            'image_size': [intrinsics.width, intrinsics.height],
            'matrix': [[intrinsics.fx, 0, intrinsics.ppx], [0, intrinsics.fy, intrinsics.ppy], [0, 0, 1]],
            'distortion': list(intrinsics.coeffs), 'model': str(intrinsics.model),
        }
        camera_path = Path(cam0_origin_path) / 'camera.json'
        camera_path.write_text(json.dumps(camera, indent=2, allow_nan=False), encoding='utf-8')
        count = 1
        logger_.info('请先点击 Capture_Video 相机窗口（不是终端），机械臂停稳后按 s 采集；在相机窗口按 q 或 Esc 退出。建议采集15～20组不同姿态。')
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame and not callback(np.asanyarray(color_frame.get_data())):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        robot_ip = get_ip()
        if not robot_ip:
            raise ConnectionError('机械臂 IP 没有 ping 通')
        with socket.create_connection((robot_ip, 8080), timeout=3.0) as client:
            if not send_cmd(client, '{"command":"set_change_work_frame","frame_name":"Base"}', get_pose=False):
                raise ConnectionError('切换 Base 工作坐标系失败，停止采集')
            displayD435()
    except KeyboardInterrupt:
        logger_.info('采集已停止')
    except (OSError, ValueError, RuntimeError, cv2.error) as exc:
        logger_.error_(f'采集失败：{exc}')
        raise SystemExit(1)
