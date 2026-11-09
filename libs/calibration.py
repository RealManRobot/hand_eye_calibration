"""两种安装方式共用的数据配对、相机位姿计算和结果检查。"""
import csv
from datetime import datetime
import json
import logging
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from libs.auxiliary import find_latest_data_folder
from save_poses import pose_to_homogeneous_matrix
from save_poses2 import inverse_transformation_matrix

ROOT = Path(__file__).resolve().parents[1]
logger = logging.getLogger(__name__)


def load_sample_pairs(directory):
    images = sorted(directory.glob('*.jpg'), key=lambda p: int(p.stem) if p.stem.isdecimal() else -1)
    if not images or any(not p.stem.isdecimal() for p in images):
        raise ValueError('数据目录需要包含以数字编号的 jpg 图片')
    if len({int(p.stem) for p in images}) != len(images):
        raise ValueError('图片编号重复，无法确定图片和位姿的配对')
    poses = []
    with (directory / 'poses.txt').open(encoding='utf-8') as stream:
        for line, row in enumerate(csv.reader(stream), 1):
            try:
                pose = np.asarray(row, dtype=float)
            except ValueError as exc:
                raise ValueError(f'poses.txt 第 {line} 行不是有效数值') from exc
            if pose.shape != (6,) or not np.isfinite(pose).all():
                raise ValueError(f'poses.txt 第 {line} 行必须包含六个有限数值')
            poses.append(pose)
    if len(images) != len(poses):
        raise ValueError(f'图片 {len(images)} 张，位姿 {len(poses)} 条，无法可靠配对；请检查原始数据')
    # 旧采集程序可能产生编号间断；位姿行对应实际保存图片的先后顺序。
    return list(zip(images, poses))


def check_motions(tools, targets):
    vectors = []
    for i in range(len(tools)):
        for j in range(i + 1, len(tools)):
            robot = Rotation.from_matrix(tools[j][:3, :3].T @ tools[i][:3, :3]).as_rotvec()
            camera = Rotation.from_matrix(targets[j][:3, :3] @ targets[i][:3, :3].T).as_rotvec()
            norms = [2 * np.sin(np.linalg.norm(v) / 2) for v in (robot, camera)]
            # 与 OpenCV 4.10 Tsai 使用的有效相对旋转范围一致。
            if all(0.3 <= n <= 1.7 for n in norms):
                vectors.append(robot)
    if len(vectors) < 2:
        raise ValueError('有效旋转变化不足，请增加不同方向的旋转后重新采集')
    singular = np.linalg.svd(vectors, compute_uv=False)
    if singular[1] < singular[0] * 0.01:
        raise ValueError('旋转方向过于单一，请增加绕不同轴的姿态变化')


def calibrate(directory=None, eye_to_hand=False):
    if directory is None:
        parent = ROOT / 'eye_hand_data'
        latest = find_latest_data_folder(str(parent)) if parent.is_dir() else None
        if latest is None:
            raise ValueError('未找到采集数据，请先运行 collect_data.py')
        directory = parent / latest
    directory = Path(directory).resolve()
    if not directory.is_dir():
        raise ValueError(f'数据目录不存在：{directory}')
    result_path = directory / ('result_to_hand.json' if eye_to_hand else 'result_in_hand.json')
    temporary = result_path.with_suffix('.tmp')
    # 提前失败也不能留下上一次的 candidate，被误认为本次计算成功。
    temporary.write_text(json.dumps({'status': 'failed',
        'created_at': datetime.now().astimezone().isoformat(timespec='seconds'),
        'reason': '本次计算未完成，请查看终端错误信息'}, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(result_path)
    logger.info('使用数据目录：%s', directory)
    with (ROOT / 'config.yaml').open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get('checkerboard_args'), dict):
        raise ValueError('config.yaml 缺少 checkerboard_args 配置')
    board = config['checkerboard_args']
    xx, yy, length = board['XX'], board['YY'], board['L']
    if (type(xx) is not int or type(yy) is not int or min(xx, yy) < 2
            or not np.isfinite(length) or length <= 0):
        raise ValueError('棋盘内角点数必须为大于1的整数，格长必须为正数，单位为米')
    limits = config.get('calibration_args', {})
    if not isinstance(limits, dict):
        raise ValueError('calibration_args 必须为误差限值配置')
    max_reprojection = float(limits.get('max_reprojection_error_px', 2.0))
    max_position = float(limits.get('max_position_error_mm', 10.0))
    max_rotation = float(limits.get('max_rotation_error_deg', 3.0))
    if not all(np.isfinite(v) and v > 0 for v in (max_reprojection, max_position, max_rotation)):
        raise ValueError('标定误差限值必须为有限正数')
    pairs = load_sample_pairs(directory)
    obj = np.zeros((xx * yy, 3), np.float32)
    obj[:, :2] = np.mgrid[0:xx, 0:yy].T.reshape(-1, 2) * length
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    accepted, rejected = [], []
    size = None
    for image_path, pose in pairs:
        image = cv2.imread(str(image_path))
        if image is None:
            raise ValueError(f'图片无法读取：{image_path.name}')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if size is not None and size != gray.shape[::-1]:
            raise ValueError('同一批图片的分辨率必须一致')
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (xx, yy), None)
        if not found:
            rejected.append({'image': image_path.name, 'reason': '未检测到完整棋盘角点'})
            continue
        # 初始角点可能偏移超过5像素，窗口需覆盖真实交点。
        corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
        accepted.append((image_path.name, pose, corners))
    if len(accepted) < 6:
        raise ValueError(f'有效配对仅 {len(accepted)}/{len(pairs)} 组，至少需要6组，建议15～20组；'
                         f'未检测到角点：{[item["image"] for item in rejected]}')

    camera_path = directory / 'camera.json'
    if camera_path.exists():
        camera = json.loads(camera_path.read_text(encoding='utf-8'))
        matrix = np.asarray(camera['matrix'], dtype=float)
        dist = np.asarray(camera['distortion'], dtype=float).reshape(-1)
        if (list(size) != camera['image_size'] or matrix.shape != (3, 3)
                or dist.size not in (4, 5, 8, 12, 14) or not np.isfinite(matrix).all()
                or not np.isfinite(dist).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], [0, 0, 1])):
            raise ValueError('相机内参无效或与图片分辨率不匹配，请使用原始图片和对应内参')
        model = camera.get('model', 'brown_conrady')
        if not isinstance(model, str):
            raise ValueError('相机畸变模型名称无效')
        model = model.removeprefix('distortion.')
        if model == 'none':
            dist = np.zeros_like(dist)
        elif model != 'brown_conrady' and np.any(dist):
            raise ValueError(f'相机畸变模型 {model} 不能直接作为 OpenCV Brown 畸变参数使用')
        rvecs, tvecs = [], []
        for name, _, corners in accepted:
            ok, rvec, tvec = cv2.solvePnP(obj, corners, matrix, dist)
            if not ok:
                raise ValueError(f'{name} 的标定板位姿求解失败')
            rvecs.append(rvec)
            tvecs.append(tvec)
        camera_source = 'camera.json'
    else:
        logger.warning('旧数据没有 camera.json，将由本批图片估计内参；新采集数据会固定内参')
        _, matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
            [obj] * len(accepted), [item[2] for item in accepted], size, None, None)
        camera_source = 'calibrateCamera'

    tools, targets, used, reprojections = [], [], [], []
    for (name, pose, corners), rvec, tvec in zip(accepted, rvecs, tvecs):
        if not np.isfinite(rvec).all() or not np.isfinite(tvec).all() or tvec[2, 0] <= 0:
            rejected.append({'image': name, 'reason': '标定板位姿无效'})
            continue
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, dist)
        error = float(np.sqrt(np.mean(np.sum((corners - projected) ** 2, axis=2))))
        if not np.isfinite(error) or error > max_reprojection:
            rejected.append({'image': name, 'reason': f'重投影误差过大：{error:.3f}px'})
            continue
        tool = pose_to_homogeneous_matrix(pose)
        tools.append(inverse_transformation_matrix(tool) if eye_to_hand else tool)
        target = np.eye(4)
        target[:3, :3] = cv2.Rodrigues(rvec)[0]
        target[:3, 3] = np.asarray(tvec).reshape(3)
        targets.append(target)
        used.append(name)
        reprojections.append(error)
    for item in rejected:
        logger.warning('剔除 %s：%s', item['image'], item['reason'])
    if len(tools) < 6:
        raise ValueError(f'质量检查后仅剩 {len(tools)} 组有效配对，至少需要6组')
    check_motions(tools, targets)
    rotation, translation = cv2.calibrateHandEye(
        [t[:3, :3] for t in tools], [t[:3, 3] for t in tools],
        [t[:3, :3] for t in targets], [t[:3, 3] for t in targets],
        method=cv2.CALIB_HAND_EYE_TSAI)
    if (not np.isfinite(rotation).all() or not np.isfinite(translation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)):
        raise ValueError('手眼求解返回无效变换，请检查采样姿态和数据配对')
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation.reshape(3)
    # 眼在手上检查板到基座；眼在手外的 tools 已取逆，检查板到末端。
    closures = np.asarray([tool @ transform @ target for tool, target in zip(tools, targets)])
    positions = closures[:, :3, 3]
    position_errors = np.linalg.norm(positions - positions.mean(axis=0), axis=1) * 1000
    rotations = Rotation.from_matrix(closures[:, :3, :3])
    rotation_errors = np.rad2deg((rotations.mean().inv() * rotations).magnitude())
    quality = {
        'position_rms_mm': float(np.sqrt(np.mean(position_errors ** 2))),
        'position_max_mm': float(position_errors.max()),
        'rotation_rms_deg': float(np.sqrt(np.mean(rotation_errors ** 2))),
        'rotation_max_deg': float(rotation_errors.max()),
        'reprojection_rms_px': float(np.sqrt(np.mean(np.square(reprojections)))),
        'reprojection_max_image_rms_px': float(max(reprojections)),
    }
    passed = quality['position_max_mm'] <= max_position and quality['rotation_max_deg'] <= max_rotation
    result = {
        'created_at': datetime.now().astimezone().isoformat(timespec='seconds'),
        'status': 'candidate' if passed else 'rejected',
        'direction': 'camera_to_base' if eye_to_hand else 'camera_to_end',
        'translation_unit': 'm', 'quaternion_order': 'xyzw',
        'matrix': transform.tolist(), 'quaternion': Rotation.from_matrix(rotation).as_quat().tolist(),
        'camera_source': camera_source, 'camera_matrix': matrix.tolist(),
        'distortion': dist.reshape(-1).tolist(), 'image_size': list(size), 'board': board,
        'accepted_images': used, 'rejected_images': rejected,
        'reprojection_errors_px': dict(zip(used, reprojections)), 'quality': quality,
        'limits': {'max_reprojection_error_px': max_reprojection,
                   'max_position_error_mm': max_position, 'max_rotation_error_deg': max_rotation},
    }
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(result_path)
    logger.info('有效配对 %s/%s', len(used), len(pairs))
    logger.info('位置残差：RMS %.3f mm；最大 %.3f mm（最大值限值 %.3f mm）',
                quality['position_rms_mm'], quality['position_max_mm'], max_position)
    logger.info('姿态残差：RMS %.3f °；最大 %.3f °（最大值限值 %.3f °）',
                quality['rotation_rms_deg'], quality['rotation_max_deg'], max_rotation)
    logger.info('重投影误差：RMS %.3f px；单图RMS最大 %.3f px（单图限值 %.3f px）',
                quality['reprojection_rms_px'], quality['reprojection_max_image_rms_px'], max_reprojection)
    logger.info('结果状态：%s；结果文件：%s', result['status'], result_path)
    if not passed:
        raise ValueError('固定关系误差超限，结果已标记 rejected；请检查配对、棋盘参数和采样质量')
    return rotation, translation
