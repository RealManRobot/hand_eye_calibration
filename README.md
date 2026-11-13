## 概述

手眼标定通常用于机器人和计算机视觉领域，特别是在需要精确控制机械臂与环境交互的场景中去，手眼标定是将机械手和摄像机的坐标系统一起来，解决相机与机械手之间的坐标转换关系，让机械手能精确抓取到相机定位的目标。

手眼系统，是手（机械臂）和眼（相机）的关系，当眼（相机）看到一个物体的时候，需要告诉手（机械臂）物体的位置在哪里，物体在眼（相机）的位置确定了，如果此时有了眼（相机）和手（机械臂）的关系，我们就能得到物体在手（机械臂）的位置了。

当需要进行机器人抓取物体、动态环境交互、精密测量与检测、视觉伺服控制等多个场景时，都需要进行手眼标定。

手眼标定之后，除非相机和机械臂的相对位置发生变动，则不需要重新标定。下面的标定方法适用于**正装、侧装和倒装**。



手眼标定有两种情况：

- 一种是相机固定在机械臂上

​		Eye-in-Hand系统:相机安装在机械臂末端，在机械臂移动过程中随着机械臂一起运动。

![image](picture/f6c716fb-c8d2-4adc-b3da-a86c6b1e78d0.png)



- 第二种是相机固定在机械臂之外某处

  ​	![44776e79-47f7-4de2-9ef2-172b654169d5](picture/44776e79-47f7-4de2-9ef2-172b654169d5-17291349013411.png)



## 原理

下文采用列向量，${}^{a}_{b}M$ 或 $H^{a}_{b}$ 表示从坐标系 $b$ 到坐标系 $a$ 的齐次变换，矩阵从右向左作用。三维点的齐次坐标记为 $\widetilde{p} = [x, y, z, 1]^{\mathsf{T}}$。

### 1.眼在手上（eye-in-hand)

​		**对于eye-in-hand情况，机器人手眼标定即标定得到相机和机械臂末端之间的坐标变换关系:**

![image](picture/f6c716fb-c8d2-4adc-b3da-a86c6b1e78d0.png)

对目标点的空间三维坐标进行变换的过程中，首先遇到的问题就是机械臂末端坐标系与相机坐标系之间的位置变换关系，也就是机械臂的手眼位置关系，也是手眼标定最后计算的结果，该关系用符号X表示，可以用方程AX=XB求解。其中A表示相邻两次运动时**机械臂末端的变换关系**；B表示相邻两次运动时**相机坐标的相对运动**。

如图1所示，为眼在手上，也即eye-in-hand。（此处仅是为了演示效果图，而非真实的实验场景）。相机固定在机械臂末端，会随着机械臂的运动而运动。



![图1 眼在手上](picture/1b3bb9f5348fe9f1dd4ae02afed614e9.png)

- A：机械臂末端在机械臂坐标系下的位姿，通过机械臂API获取。（已知）。

$$
{}^{base}_{end}M
$$
  
- B：相机在机器人末端坐标系下的位姿，这个变换是固定的，只要知道这个变换，我们就可以随时计算相机的实际位置，所以这就是我们想求的东西。（未知，待求）

$$
{}^{end}_{camera}M
$$
  
- C：标定板在相机坐标系下的位姿，由棋盘角点和相机内参通过 `solvePnP` 求得。

$$
{}^{camera}_{board}M
$$

- D：标定板在机器人坐标系下的位姿。在标定过程中，只有机械臂末端在动，标定板和机械臂基座不动，这个位姿关系是固定不变的。


$$
{}^{base}_{board}M
$$

所以我们只要计算得到B变换，那么标定板在机械臂坐标系下的位姿D也就自然得到了:

$$
{}^{base}_{board}M = {}^{base}_{end}M \cdot {}^{end}_{camera}M \cdot {}^{camera}_{board}M
$$

如图2所示，我们让机械臂运动两个位置，保证这两个位置下都可以看到标定板，然后构建空间变换回路：

![图2 机械臂运动到两个位置，构建变换回路](picture/29fb4d433468f12530eca3e2a563da72.png)

$$
A_1 \cdot B \cdot C_1 = A_2 \cdot B \cdot C_2
$$

$$
\left( A_2^{-1} \cdot A_1 \right) \cdot B = B \cdot \left( C_2 \cdot C_1^{-1} \right)
$$

等同于下面的公式：



$$
{}^{base}_{end}M_1 \cdot {}^{end}_{camera}M \cdot {}^{camera}_{board}M_1 = {}^{base}_{end}M_2 \cdot {}^{end}_{camera}M \cdot {}^{camera}_{board}M_2
$$

$$
\left( {}^{base}_{end}M_2 \right)^{-1} \cdot {}^{base}_{end}M_1 \cdot {}^{end}_{camera}M = {}^{end}_{camera}M \cdot {}^{camera}_{board}M_2 \cdot \left( {}^{camera}_{board}M_1 \right)^{-1}
$$

将上式中的固定变换 $B$ 记为 $X$，即可整理成相对运动满足的 **AX=XB** 形式。其中 $X = {}^{end}_{camera}M$ 是一个4×4齐次变换矩阵：

$$
X = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}
$$

手眼标定的目的就是为了计算出X

​	

### 2 .眼在手外（eye-to-hand)

### ![44776e79-47f7-4de2-9ef2-172b654169d5](picture/44776e79-47f7-4de2-9ef2-172b654169d5-17291482180503.png)

**眼在手外**标定时**固定机械臂基座和相机**，将**标定板固定在机械臂末端**，所以标定过程中**标定板与机械臂末端的关系固定不变，以及相机与机器人基座标的关系固定不变**

标定的目标：相机到机械臂基座坐标系的变换矩阵

$$
{}^{base}_{camera}M
$$

实现方法：1.把标定板固定在机械臂末端

​					2.移动机械臂末端，使用相机拍摄不同机械臂姿态下的标定板图片n张 (10~20)

每次采集图片和机械臂位姿，都存在下面等式：

$$
{}^{end}_{board}M = {}^{end}_{base}M \cdot {}^{base}_{camera}M \cdot {}^{camera}_{board}M
$$


其中：

| 符号               | 描述                         |
| ------------------ | ---------------------------- |
|${}^{end}_{board}M$  | 标定板到机械臂末端的变换矩阵（因为标定过程中标定板固定在机械臂末端，标定板到机械臂末端的变化矩阵不变） |
|${}^{end}_{base}M$ | 可以通过机械臂末端位姿算出   |
|${}^{base}_{camera}M$ | 手眼标定需要求的             |
|${}^{camera}_{board}M$ | 通过相机标定方法得到         |



则可以得到如下等式：

$$
{}^{end}_{base}M_1 \cdot {}^{base}_{camera}M \cdot {}^{camera}_{board}M_1 = {}^{end}_{base}M_2 \cdot {}^{base}_{camera}M \cdot {}^{camera}_{board}M_2
$$

$$
\left( {}^{end}_{base}M_2 \right)^{-1} \cdot {}^{end}_{base}M_1 \cdot {}^{base}_{camera}M = {}^{base}_{camera}M \cdot {}^{camera}_{board}M_2 \cdot \left( {}^{camera}_{board}M_1 \right)^{-1}
$$

$$
\vdots
$$

$$
\left( {}^{end}_{base}M_n \right)^{-1} \cdot {}^{end}_{base}M_{n-1} \cdot {}^{base}_{camera}M = {}^{base}_{camera}M \cdot {}^{camera}_{board}M_n \cdot \left( {}^{camera}_{board}M_{n-1} \right)^{-1}
$$



这也是是一个典型的**AX=XB**问题，而且根据定义，其中X是一个4X4齐次变换矩阵，其中R是相机到机械臂基坐标系的旋转矩阵，t是相机到机械臂基坐标系的平移向量：

$$
X = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}
$$

手眼标定的目的就是为了计算出X。



## 关键代码解释

### 代码结构

```
---eye_hand_data  各批次的采集数据和标定结果

---libs

​         ---auxiliary.py 程序用到的一些辅助的包

​         ---log_settings.py 日志包

​         ---calibration.py 两种标定方式共用的求解与质量检查

---config.yaml 棋盘格参数和误差限值

---collect_data.py 手眼标定时采集数据程序

---compute_in_hand.py 眼在手上标定计算程序

---compute_to_hand.py 眼在手外标定计算程序

---requirements.txt 环境依赖文件

---save_poses.py 计算依赖文件

---save_poses2.py 计算依赖文件
```



### 采集与计算

在项目目录运行 `python collect_data.py`。启动采集时，脚本将控制器的工作坐标系切到 `Base`。标定期间保持同一工具坐标系。点击 `Capture_Video` 预览窗口，等机械臂停稳后按 `s`。脚本使用当前画面，再查询机械臂末端位姿。在预览窗口按 `q` 或 `Esc` 退出。预览放大两倍，保存的图像保持采集分辨率。

每批数据保存到程序目录的 `eye_hand_data` 下，包含按数字编号的图片、`poses.txt` 和 `camera.json`。图片按数字顺序与位姿逐行对应；请勿单独删除图片、删除位姿行或修改图片编号。通信失败或保存失败的样本不计数。

`camera.json` 保存当前彩色流分辨率、内参和畸变参数，计算时使用这些参数，通过 `solvePnP` 求标定板位姿。旧数据没有该文件时，仍使用 `calibrateCamera` 从本批图片估计内参，并显示提示。不要把其他相机或其他分辨率的内参复制到数据目录。

按安装方式选择求解脚本，并指定对应的采集目录。两种安装方式分别采集。省略目录时，脚本按目录名选择最新一批数据。

| 安装方式 | 运行命令 | 输出 |
| --- | --- | --- |
| 眼在手上 | `python compute_in_hand.py "eye_hand_data/<眼在手上批次目录>"` | `result_in_hand.json`，相机到末端 |
| 眼在手外 | `python compute_to_hand.py "eye_hand_data/<眼在手外批次目录>"` | `result_to_hand.json`，相机到基座 |

求解使用当前 `config.yaml`。复算旧批次前核对该批棋盘参数，重复求解会覆盖该批目录里的同名结果。

程序按实际文件名读取图片。检测角点或图片质量检查失败时，同步剔除对应的机械臂位姿。至少需要6组有效配对，建议采集15～20组，并包含绕不同轴的充分旋转。

计算逻辑共用 `libs/calibration.py`。眼在手上使用末端到基座的位姿，眼在手外使用它的逆变换。`poses.txt` 每行记录 `x,y,z,rx,ry,rz`，单位为米和弧度。这里的末端指采集位姿使用的当前工具坐标系。

结果保存在本批数据的 `result_in_hand.json` 或 `result_to_hand.json`，记录矩阵、变换方向、单位、有效图片编号、被剔除图片及原因、重投影误差和固定关系误差。四元数顺序为 `[x,y,z,w]`，平移单位为米。

计算结束后，终端同时显示位置残差（毫米）、姿态残差（度）和重投影误差（像素）的RMS、最大值及对应限值；残差汇总也保存在结果文件的 `quality` 字段。超限时同样输出这些数值，结果仍标记为 `rejected`。

当前 `config.yaml` 对单张图像使用 **2 px** 的重投影 RMS 限值，超限图像和对应的位姿会一并剔除。固定关系最大位置残差的限值是 **12 mm**，最大姿态残差的限值是 **3°**。后两项任一超限时，结果标记为 `rejected`。求解提前失败时标记为 `failed`，通过检查时标记为 `candidate`。（注：限值仅仅代表计算出来的残差过大，不建议使用，原因有：旋转角度不足、保存的姿态难以满足精准计算的要求、臂的精度和状态等等）

相机固定内参路径支持 OpenCV Brown 畸变参数及无畸变图像；其他 RealSense 非零畸变模型会明确拒绝，不能将其系数直接当作 Brown 参数使用。

#### 手眼标定计算

```python
R, t = cv2.calibrateHandEye(
    R_tool, t_tool, rvecs, tvecs, method=cv2.CALIB_HAND_EYE_TSAI
)

```

- 使用OpenCV的 `calibrateHandEye` 函数进行手眼标定

  - 在compute_in_hand.py中计算**相机**相对于**机械臂末端**的旋转矩阵 **R_cam2end**和平移向量 **T_cam2end**。

    ```
    void
    cv::calibrateHandEye(InputArrayOfArrays 	R_end2base,
                         InputArrayOfArrays 	T_end2base, 
                         InputArrayOfArrays 	R_board2cam,
                         InputArrayOfArrays 	T_board2cam,
                         OutputArray 	        R_cam2end,
                         OutputArray 	        T_cam2end, 
                         HandEyeCalibrationMethod method = CALIB_HAND_EYE_TSAI)	
    
    ```

    

  - 在compute_to_hand.py中计算**相机**相对于**机械臂基座**的旋转矩阵 **R_cam2base** 和平移向量**T_cam2base**。

    ```
    void
    cv::calibrateHandEye(InputArrayOfArrays 	R_base2end
                         InputArrayOfArrays 	T_base2end
                         InputArrayOfArrays 	R_board2cam
                         InputArrayOfArrays 	T_board2cam
                         OutputArray 	        R_cam2base
                         OutputArray 	        T_cam2base
                         HandEyeCalibrationMethod method = CALIB_HAND_EYE_TSAI)	
    
    ```

    

- 采用了 `CALIB_HAND_EYE_TSAI` 方法，这是常用的手眼标定算法



### `save_poses.py` 关键代码解释

#### 1. **定义欧拉角转换为旋转矩阵的函数**

```python
def euler_angles_to_rotation_matrix(rx, ry, rz):
    # 计算旋转矩阵
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx

    return R

```

- 定义了将欧拉角（绕X、Y、Z轴的旋转）转换为旋转矩阵的函数。
- 欧拉角以弧度表示。旋转矩阵为 `Rz @ Ry @ Rx`，作用于列向量时从右到左应用。

#### 2. **将位姿转换为齐次变换矩阵**

```python
def pose_to_homogeneous_matrix(pose):
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)

    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t[:, 0]

    return H

```

- 将位姿（位置和姿态）转换为4x4的齐次变换矩阵，以便进行矩阵运算。
- 位置 `(x, y, z)` 作为平移向量，姿态 `(rx, ry, rz)` 作为旋转欧拉角。
- 生成的齐次变换矩阵 `H` 将用于描述**机械臂末端相对于基座的旋转变换**。

求解脚本直接从 `poses.txt` 读取位姿，并将标定矩阵写入结果 JSON。

## 标定过程

### 1.环境要求

#### Python 环境准备

推荐使用 **Python 3.11**。当前锁定的 RealSense Python 包提供 Linux 和 Windows x86-64 的 Python 3.11 安装包。依赖版本见 [requirements.txt](requirements.txt)。[RealSense 发布文件](https://pypi.org/project/pyrealsense2/2.55.1.6486/#files)

Ubuntu

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Windows PowerShell

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

#### 设备准备

- 机械臂：RM75 RM65  RM63 GEN72 
- 相机：Intel RealSense D435，采集脚本请求 `640 × 480 @ 30 FPS` 的彩色流

- 摄像头专用数据线

- 网线

- 标定板（1或2）

  1. 打印纸质标定板
     ![标定版图片__00(A)](picture/ba734972b343537d8d040a1c85d07d3e.png)  
A4纸打印L约27.72mm（计算时以实际长度为准）
     ![标定板图片_00(1)](picture/标定板图片_00(1).png)
A4纸打印L约20. 21mm（计算时以实际长度为准）
  2. 淘宝上搜索“标定板棋盘格”购买







### 2.标定过程

#### 参数配置

在 [config.yaml](config.yaml) 的 `checkerboard_args` 中填写实际棋盘参数。

| 参数 | 含义 | 当前配置 |
| --- | --- | --- |
| `XX` | 横向内角点数，比横向方格数少一 | 8 |
| `YY` | 纵向内角点数，比纵向方格数少一 | 5 |
| `L` | 单格实测边长，单位为米 | 0.02772 |

打印后测量连续多格的总长，再除以格数。打印缩放会改变实际格长。

![棋盘格内角点计数示意](picture/image-20241016181226851.png)

#### 眼在手上

##### 采集数据

(1).相机连接线连接**电脑**和D435相机，网线连接**电脑**和机械臂

(2).电脑的ip和机械臂设置为同一网段

如果机械臂IP为192.168.1.18 ，则将电脑 ip地址设为1网段

![image-20241018143659263](picture/image-20241018143659263.png)

如果机械臂 IP 为 `192.168.10.18`，将电脑设到对应网段。脚本依次探测 `192.168.1.18` 和 `192.168.10.18`，连接第一个可达地址的 TCP `8080` 端口。

(3).标定板放置在平面上，相机固定在机械臂末端，相机对准标定板

​		标定过程中，标定板是**固定放置**在机械臂工作区内的，这个固定位置需要保证安装在**机械臂末端的相机能从不同的视角**观测到它，标定板的确切位置不重要，因为不必知道其相对于机械臂基座的位姿。但是标定板在标定期间应**保持固定，不得移动**。

(4).运行脚本`collect_data.py`，出现一个弹窗

(5). 调整机械臂末端姿态，让相机拍到清晰、完整的棋盘。

​		**注意：**

- 使出现在相机视野里的**标定板**和**相机镜面**呈现**一定角度**。

  ![image-20241101112801145](picture/image-20241101112801145.png)

  下面的拍照姿势**不对**：
  ![image-20241101112816914](picture/image-20241101112816914.png)

​				

(6). 点击预览窗口，等机械臂停稳后按 `s` 采集。

(7)移动15-20次机械臂，重复步骤(5)(6)，采集不同机械臂姿态下的标定板图片15-20张左右

​	**注意：**

​			移动**机械臂末端旋转轴**，每次旋转的角度尽量的大（大于30°）

​			确保在三个轴（X、Y、Z）上都有足够的旋转角度变化。

​			（可以先绕着机械臂末端z轴旋转多个角度拍摄多副图片，然后绕着x轴旋转）

​			![WPS拼图1](picture/WPS拼图1.png)

##### 计算标定结果

选择眼在手上的采集目录，运行 `compute_in_hand.py`。

​        得出**相机坐标系**相对于**机械臂末端**坐标系的**旋转矩阵**和**平移向量**

#### 眼在手外

##### 采集数据

(1).相机连接线连接**电脑**和D435相机，网线连接**电脑**和机械臂

(2).电脑的ip和机械臂设置为同一网段

​	如果机械臂IP为192.168.1.18 ，则将电脑 ip地址设为1网段

​    如果机械臂的IP为192.168.10.18，则将电脑的IP设置为10网段

(3).将标定板（**打印纸质较小的板子，方便固定**）**固定在机械臂末端，相机固定不动**，移动机械臂末端，使标定板出现在相机视野里

​       标定过程中，标定板安装在**机械臂末端执行器上**并随机械臂移动。可以直接固定在工具法兰上或由夹具固定安装，安装的确切位置不重要，因为**不必知道标定板和末端执行器**的相对位姿。重要的是标定板在运动过程中不会出现**相对于工具法兰或夹具的位移**，它必须被良好地固定住或被夹具紧紧地抓住。建议使用由刚性材料制成的安装支架。

(4).运行脚本`collect_data.py`，出现一个弹窗

(5). 调整机械臂末端姿态，让相机拍到清晰、完整的棋盘。

​		**注意：**

​				使出现在相机视野里的标定板和相机镜面呈现一定角度。

(6). 点击预览窗口，等机械臂停稳后按 `s` 采集。

(7)移动15-20次机械臂，重复步骤(5)(6)，采集不同机械臂姿态下的标定板图片15-20张左右

​			移动**机械臂末端旋转轴**，每次旋转的角度尽量的大（大于30°）

​			确保在三个轴（X、Y、Z）上都有足够的旋转角度变化。



##### 计算标定结果

选择眼在手外的采集目录，运行 `compute_to_hand.py`。

得出**相机坐标系**相对于**机械臂基坐标系**的**旋转矩阵**和**平移向量**



**平移向量**的单位是米



#### 误差范围

`candidate` 表示通过同一批样本的一致性检查。实际定位精度需用工作区内的独立实测点验证，测点应覆盖实际使用的距离和姿态。

## 标定过程中可能出现的问题

### 问题1

在计算采集到的数据时

即执行下面脚本时：

```bash
python compute_in_hand.py
```

或

```bash
python compute_to_hand.py
```

可能出现下面问题

问题描述：[ERROR:0@1.418] global calibration_handeye.cpp:335 calibrateHandEyeTsai Hand-eye calibration failed! Not enough informative motions--include larger rotations.



**问题原因**：采集的图片旋转量不足，特别是缺少足够大的旋转运动。

手眼标定需要机器人在空间中执行一系列运动，以获取相机相对于机器人末端的精确关系。若机器人的运动数据中缺乏足够的旋转变化，尤其是在各个轴向上的显著旋转，标定算法就无法准确计算出手眼关系。



**解决方案：**

1. **增加旋转运动：**

- 在数据采集过程中，让机器人执行更大的旋转运动。确保在三个轴（X、Y、Z）上都有足够的旋转角度变化。
- 例如，旋转角度最好能超过30度，这样能提供丰富的运动信息。

2. **多样化运动姿态：**

- 采集数据时，保证机器人末端执行多样化的姿态变化，包括平移和旋转。
- 避免只在小范围内运动或只在某一轴上运动。

3. **增加数据采集次数：**

- 求解至少需要 6 组通过检查的样本，建议采集 15～20 组不同姿态，留出剔除余量。
- 更多的数据能提高标定的稳定性和准确性。

## 手眼标定结果如何使用

下文用 `T_目标_来源` 表示从来源坐标系到目标坐标系的变换。三维点和物体位姿都应位于标定所用的相机光学坐标系，长度单位为米。像素坐标需要结合深度和相机内参转换为三维点。

先加载与安装方式匹配的结果 JSON。通过独立测点验证后，再将矩阵用于实际定位。

```python
import json
from pathlib import Path

import numpy as np

from save_poses import pose_to_homogeneous_matrix


def load_calibration(result_path, direction):
    result = json.loads(Path(result_path).read_text(encoding="utf-8"))
    if result["status"] != "candidate":
        raise ValueError("请选择通过质量检查的结果")
    if result["direction"] != direction:
        raise ValueError(f"标定方向应为 {direction}")
    return np.asarray(result["matrix"], dtype=float)
```

### 眼在手上

#### 理论

从一个**不涉及相机**的机械臂开始。它的两个主要坐标系是：

​	1.机械臂基坐标系

   2.末端执行器坐标系

![../../../_images/hand-eye-robot-ee-robot-base-coordinate-systems.png](picture/hand-eye-robot-ee-robot-base-coordinate-systems-17301939637152.png)






为了捡取物品，机械臂需要知道**物品相对于机械臂基坐标系**的位姿。通过这些信息以及机器人相关的几何知识，即可算出末端执行器/夹具朝物体移动的关节角度。

![../../../_images/hand-eye-robot-robot-to-object.png](picture/hand-eye-robot-robot-to-object.png)

**物体相对相机坐标系的位姿**可以通过**模型识别**得到，为了使机械臂能够拾取物体，需要将物体的位姿从**相机的坐标系**转换到**机械臂的基坐标系**。

![../../../_images/hand-eye-robot-ee-robot-base-coordinate-systems-with-camera.png](picture/hand-eye-robot-ee-robot-base-coordinate-systems-with-camera.png)

在这种情况下，坐标转换是间接完成的：

$$
H^{ROB}_{OBJ} = H^{ROB}_{EE} \cdot H^{EE}_{CAM} \cdot H^{CAM}_{OBJ}
$$

末端执行器相对于机械臂基座的位姿

$$
H^{ROB}_{EE}
$$

是已知的，通过机械臂API可以获取得到，相机相对于末端执行器的位姿

$$
H^{EE}_{CAM}
$$

由手眼标定得到。

![img](picture/hand-eye-eye-in-hand-all-poses.png)



假如物体在相机坐标系是一个3D点或者一个位姿，那么下面说明物体位置作为一个3D点或者一个位姿**从相机坐标系转换到机械臂基坐标系**的数学理论。

![../../../_images/hand-eye-eye-in-hand-all-poses.png](picture/hand-eye-eye-in-hand-all-poses-17302010389227.png)

机械臂的位姿用齐次变换矩阵表示。

以下方程描述了如何将单个3D点从相机坐标系转换到机械臂基坐标系：

$$
\widetilde{p}^{ROB} = H^{ROB}_{EE} \cdot H^{EE}_{CAM} \cdot \widetilde{p}^{CAM}
$$

$$
\begin{bmatrix} x^r \\ y^r \\ z^r \\ 1 \end{bmatrix} = \begin{bmatrix} R_e^r & t_e^r \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} R_c^e & t_c^e \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} x^c \\ y^c \\ z^c \\ 1 \end{bmatrix}
$$

如果要将物体位姿从相机坐标系转换到机械臂基坐标系。

$$
H^{ROB}_{OBJ} = H^{ROB}_{EE} \cdot H^{EE}_{CAM} \cdot H^{CAM}_{OBJ}
$$

$$
\begin{bmatrix} R_o^r & t_o^r \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} R_e^r & t_e^r \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} R_c^e & t_c^e \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} R_o^c & t_o^c \\ 0 & 1 \end{bmatrix}
$$


上式给出物体在基座下的位姿 `T_base_object`。如果用它生成抓取目标，还要右乘抓取时 TCP 相对于物体的变换 `T_object_tcp`，其中包含夹爪朝向和抓取偏移。

```text
T_base_tcp = T_base_object @ T_object_tcp
```

两种安装方式都使用同样的抓取变换。



#### 代码

使用与图像对应时刻的末端位姿 `end_pose = [x, y, z, rx, ry, rz]`，位置为米、姿态为弧度，工作坐标系为 `Base`。使用标定时的同一工具坐标系。

```python
def wrist_camera_to_base(result_path, end_pose):
    T_end_camera = load_calibration(result_path, "camera_to_end")
    T_base_end = pose_to_homogeneous_matrix(end_pose)
    return T_base_end @ T_end_camera
```

- **物体是一个三维点**。在相机点的末尾补 `1`，使矩阵同时应用旋转和平移。

  ```python
  def wrist_point_to_base(result_path, end_pose, camera_point):
      T_base_camera = wrist_camera_to_base(result_path, end_pose)
      return (T_base_camera @ np.append(camera_point, 1.0))[:3]
  ```

- **物体有完整位姿**。传入物体到相机的 4 × 4 变换，返回物体到基座的变换。

  ```python
  def wrist_object_to_base(result_path, end_pose, T_camera_object):
      T_base_camera = wrist_camera_to_base(result_path, end_pose)
      return T_base_camera @ T_camera_object
  ```

### 眼在手外

#### 理论

相机可以通过模型获取物体在相机坐标系的里的位姿，**物体相对于机械臂的位姿**通过相机相对于机械臂基坐标系的位姿和物体相对于相机相机坐标系的位姿通过后乘法计算得到的：

$$
H^{ROB}_{OBJ} = H^{ROB}_{CAM} \cdot H^{CAM}_{OBJ}
$$

![../../../_images/hand-eye-eye-to-hand-all-poses.png](picture/hand-eye-eye-to-hand-all-poses.png)



假如物体位置是一个3D点或者一个位姿，那么下面说明物体位置作为一个3D点或者一个位姿**从相机坐标系转换到机械臂基坐标系**的数学理论。

以下方程描述了如何将单个3D点从相机坐标系转换到机械臂基坐标系：

$$
\widetilde{p}^{ROB} = H^{ROB}_{CAM} \cdot \widetilde{p}^{CAM}
$$

$$
\begin{bmatrix} x^r \\ y^r \\ z^r \\ 1 \end{bmatrix} = \begin{bmatrix} R_c^r & t_c^r \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} x^c \\ y^c \\ z^c \\ 1 \end{bmatrix}
$$

如果要将物体位姿从相机坐标系转换到机械臂基坐标系。

$$
H^{ROB}_{OBJ} = H^{ROB}_{CAM} \cdot H^{CAM}_{OBJ}
$$

$$
\begin{bmatrix} R_o^r & t_o^r \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} R_c^r & t_c^r \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} R_o^c & t_o^c \\ 0 & 1 \end{bmatrix}
$$

#### 代码

固定相机直接使用 `result_to_hand.json` 中的相机到基座变换。相机和机械臂基座的相对位置保持固定。

- **物体是一个三维点**。

  ```python
  def fixed_point_to_base(result_path, camera_point):
      T_base_camera = load_calibration(result_path, "camera_to_base")
      return (T_base_camera @ np.append(camera_point, 1.0))[:3]
  ```

- **物体有完整位姿**。

  ```python
  def fixed_object_to_base(result_path, T_camera_object):
      T_base_camera = load_calibration(result_path, "camera_to_base")
      return T_base_camera @ T_camera_object
  ```

如果下游接口需要与 `poses.txt` 同格式的六维位姿，可以把矩阵拆为平移和欧拉角。这里的 `xyz` 欧拉角与 `save_poses.py` 中的 `Rz @ Ry @ Rx` 对应。

```python
from scipy.spatial.transform import Rotation


def matrix_to_pose(transform):
    xyz = transform[:3, 3]
    rpy = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")
    return np.concatenate((xyz, rpy))
```
