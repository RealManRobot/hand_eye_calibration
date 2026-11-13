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

### 1.眼在手上（eye-in-hand)

​		**对于eye-in-hand情况，机器人手眼标定即标定得到相机和机械臂末端之间的坐标变换关系:**

![image](picture/f6c716fb-c8d2-4adc-b3da-a86c6b1e78d0.png)

对目标点的空间三维坐标进行变换的过程中，首先遇到的问题就是机械臂末端坐标系与相机坐标系之间的位置变换关系，也就是机械臂的手眼位置关系，也是手眼标定最后计算的结果，该关系用符号X表示，可以用方程AX=XB求解。其中A表示相邻两次运动时**机械臂末端的变换关系**；B表示相邻两次运动时**相机坐标的相对运动**。

如图1所示，为眼在手上，也即eye-in-hand。（此处仅是为了演示效果图，而非真实的实验场景）。相机固定在机械臂末端，会随着机械臂的运动而运动。



![图1 眼在手上](picture/1b3bb9f5348fe9f1dd4ae02afed614e9.png)

- A：机械臂末端在机械臂坐标系下的位姿，通过机械臂API获取。（已知）。

$$ {}^{base}_{end}M $$
  
- B：相机在机器人末端坐标系下的位姿，这个变换是固定的，只要知道这个变换，我们就可以随时计算相机的实际位置，所以这就是我们想求的东西。（未知，待求）

$$ {}^{end}_{camera}M $$
  
- C：相机在标定板坐标系下的位姿，这个其实就是求解相机的外参（由相机标定求出）。
  
$$ {}^{board}_{camera}M $$

- D：标定板在机器人坐标系下的位姿。在标定过程中，只有机械臂末端在动，标定板和机械臂基座不动，这个位姿关系是固定不变的。


$$ {}^{base}_{board}M $$

所以我们只要计算得到B变换，那么标定板在机械臂坐标系下的位姿D也就自然得到了:

$$ {}^{base}_{board}M = {}^{base}_{end}M \cdot {}^{end}_{camera}M \cdot {}^{camera}_{board}M $$

如图2所示，我们让机械臂运动两个位置，保证这两个位置下都可以看到标定板，然后构建空间变换回路：

![图2 机械臂运动到两个位置，构建变换回路](picture/29fb4d433468f12530eca3e2a563da72.png)

$$ A_1 \cdot B \cdot C_1^{-1} = A_2 \cdot B \cdot C_2^{-1} $$

$$ \left( A_2^{-1} \cdot A_1 \right) \cdot B = B \cdot \left( C_2^{-1} \cdot C_1 \right) $$

等同于下面的公式：



$$ {}^{base}_{end}M_1 \cdot {}^{end}_{camera}M \cdot {}^{camera}_{board}M_1 = {}^{base}_{end}M_2 \cdot {}^{end}_{camera}M \cdot {}^{camera}_{board}M_2 $$

$$ \left( {}^{base}_{end}M_2 \right)^{-1} \cdot {}^{base}_{end}M_1 \cdot {}^{end}_{camera}M = {}^{end}_{camera}M \cdot {}^{camera}_{board}M_2 \cdot \left( {}^{camera}_{board}M_1 \right)^{-1} $$

这是一个典型的**AX=XB**问题，而且根据定义，其中X是一个4X4齐次变换矩阵：

$$ X = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} $$

手眼标定的目的就是为了计算出X

​	

### 2 .眼在手外（eye-to-hand)

### ![44776e79-47f7-4de2-9ef2-172b654169d5](picture/44776e79-47f7-4de2-9ef2-172b654169d5-17291482180503.png)

**眼在手外**标定时**固定机械臂基座和相机**，将**标定板固定在机械臂末端**，所以标定过程中**标定板与机械臂末端的关系固定不变，以及相机与机器人基座标的关系固定不变**

标定的目标：相机到机械臂基座坐标系的变换矩阵

$$ {}^{base}_{camera}M $$

实现方法：1.把标定板固定在机械臂末端

​					2.移动机械臂末端，使用相机拍摄不同机械臂姿态下的标定板图片n张 (10~20)

每次采集图片和机械臂位姿，都存在下面等式：

$$ {}^{end}_{board}M = {}^{end}_{base}M \cdot {}^{base}_{camera}M \cdot {}^{camera}_{board}M $$


其中：

| 符号               | 描述                         |
| ------------------ | ---------------------------- |
|$$^{end}_{board}M$$  | 标定板到机械臂末端的变换矩阵（因为标定过程中标定板固定在机械臂末端，标定板到机械臂末端的变化矩阵不变） |
|$${}^{end}_{base}M$$ | 可以通过机械臂末端位姿算出   |
|$${}^{base}_{camera}M$$ | 手眼标定需要求的             |
|$${}^{camera}_{board}M$$ | 通过相机标定方法得到         |



则可以得到如下等式：

**The Cauchy-Schwarz Inequality**

$$ {}^{end}_{base}M_1 \cdot {}^{base}_{camera}M \cdot {}^{camera}_{board}M_1 = {}^{end}_{base}M_2 \cdot {}^{base}_{camera}M \cdot {}^{camera}_{board}M_2 $$

$$ \left( {}^{end}_{base}M_2 \right)^{-1} \cdot {}^{end}_{base}M_1 \cdot {}^{base}_{camera}M = {}^{base}_{camera}M \cdot {}^{camera}_{board}M_2 \cdot \left( {}^{camera}_{board}M_1 \right)^{-1} $$

$$ \vdots $$

$$ \left( {}^{end}_{base}M_n \right)^{-1} \cdot {}^{end}_{base}M_{n-1} \cdot {}^{base}_{camera}M = {}^{base}_{camera}M \cdot {}^{camera}_{board}M_n \cdot \left( {}^{camera}_{board}M_{n-1} \right)^{-1} $$



这也是是一个典型的**AX=XB**问题，而且根据定义，其中X是一个4X4齐次变换矩阵，其中R是相机到机械臂基坐标系的旋转矩阵，t是相机到机械臂基坐标系的平移向量：

$$ X = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} $$

手眼标定的目的就是为了计算出X。



## 关键代码解释

### 代码结构

```
---eye_hand_data  眼在手上标定时采集的数据

---libs

​         ---auxiliary.py 程序用到的一些辅助的包

​         ---log_settings.py 日志包

---robotic_arm_package 机械臂python包

---collect_data.py 手眼标定时采集数据程序

---compute_in_hand.py 眼在手上标定计算程序

---compute_to_hand.py 眼在手外标定计算程序

---requirements.txt 环境依赖文件

---save_poses.py 计算依赖文件

---save_poses2.py 计算依赖文件
```



### 采集与计算

先点击 `Capture_Video` 相机预览窗口使其获得焦点，机械臂停稳后在该窗口按 `s`，不要在终端输入。标定期间不要切换工具坐标系。在相机窗口按 `q` 或 `Esc` 退出。预览画面放大两倍，保存的图片保持相机原始分辨率。

每批数据保存到程序目录的 `eye_hand_data` 下，包含按数字编号的图片、`poses.txt` 和 `camera.json`。图片按数字顺序与位姿逐行对应；请勿单独删除图片、删除位姿行或修改图片编号。通信失败或保存失败的样本不计数。

`camera.json` 保存当前彩色流分辨率、内参和畸变参数，计算时使用这些参数，通过 `solvePnP` 求标定板位姿。旧数据没有该文件时，仍使用 `calibrateCamera` 从本批图片估计内参，并显示提示。不要把其他相机或其他分辨率的内参复制到数据目录。

两个计算入口保持不变，默认使用最新一批数据，也可以明确指定数据目录：

```bash
python compute_in_hand.py /完整路径/eye_hand_data/data20261029
python compute_to_hand.py /完整路径/eye_hand_data/data20261029
```

程序按实际文件名读取图片。检测角点或图片质量检查失败时，同步剔除对应的机械臂位姿。至少需要6组有效配对，建议采集15～20组，并包含绕不同轴的充分旋转。

计算逻辑共用 `libs/calibration.py`。眼在手上使用末端到基座的位姿，眼在手外使用它的逆变换，不再通过共享的 `RobotToolPose.csv` 中转。这里的末端指采集位姿使用的当前工具坐标系。

结果保存在本批数据的 `result_in_hand.json` 或 `result_to_hand.json`，记录矩阵、变换方向、单位、有效图片编号、被剔除图片及原因、重投影误差和固定关系误差。四元数顺序为 `[x,y,z,w]`，平移单位为米。

计算结束后，终端同时显示位置残差（毫米）、姿态残差（度）和重投影误差（像素）的RMS、最大值及对应限值；残差汇总也保存在结果文件的 `quality` 字段。超限时同样输出这些数值，结果仍标记为 `rejected`。

`config.yaml` 中 `calibration_args` 的当前配置限值为：单图重投影均方根误差2像素、固定关系位置最大偏差12毫米、旋转最大偏差3度。前者用于剔除图像，后两者用于检查最终结果。超限结果标记为 `rejected`，程序以失败状态退出。计算提前失败时，结果文件标记为 `failed`，避免沿用上次的结果。通过检查的结果标记为 `candidate`，仍需用独立实测点验证精度；同批数据误差不代表实际定位精度。

相机固定内参路径支持 OpenCV Brown 畸变参数及无畸变图像；其他 RealSense 非零畸变模型会明确拒绝，不能将其系数直接当作 Brown 参数使用。

#### 手眼标定计算

```python
R, t = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, method=cv2.CALIB_HAND_EYE_TSAI)

return R,t

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
- 按照Z-Y-X的顺序进行旋转，并将各轴的旋转矩阵相乘得到最终的旋转矩阵 `R`。

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

#### 3. **保存多个矩阵到CSV文件**

```python
Copy codedef save_matrices_to_csv(matrices, file_name):
    rows, cols = matrices[0].shape
    num_matrices = len(matrices)
    combined_matrix = np.zeros((rows, cols * num_matrices))

    for i, matrix in enumerate(matrices):
        combined_matrix[:, i * cols: (i + 1) * cols] = matrix

    with open(file_name, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        for row in combined_matrix:
            csv_writer.writerow(row)
```

- 将多个矩阵横向拼接成一个大矩阵，然后保存到CSV文件中。

- 每个矩阵占用固定的列数，这样在读取时可以按照固定的间隔切分出单个矩阵。

  

## 标定过程

### 1.环境要求

#### 基础环境准备

| 项目     | 版本           |
| :------- | :------------- |
| 操作系统 | ubuntu/windows |
| Python   | 3.9及以上      |
|          |                |

#### Python环境准备

| 包            | 版本        |
| :------------ | :---------- |
| numpy         | 2.0.2       |
| opencv-python | 4.10.0.84   |
| pyrealsense2  | 2.55.1.6486 |
| scipy         | 1.13.1      |

执行下面命令在python环境中安装手眼标定程序所需要的包：

```cmd
pip install -r requirements.txt
```



#### 设备准备

- 机械臂：RM75 RM65  RM63 GEN72 
- 相机: Intel RealSense Depth Camera D435

- 摄像头专用数据线

- 网线

- 标定板（1或2）

  1. 打印纸质标定板

     ![标定板图片_00(1)](picture/标定板图片_00(1).png)

  2. 淘宝上搜索“标定板棋盘格”购买







### 2.标定过程

#### 参数配置

在配置文件（config.yaml）中设置标定板参数

![image-20250403064543828](picture/image-20250403064543828.png)

​		

config.yaml里的配置参数如下，有下面三个

​        XX:标定板的横向内角点数（该方向格子数减1），当前配置为8。例如下图长边12个格子，角点数为11；应按实际使用的标定板填写。

​		YY:标定板纵向内角点数（该方向格子数减1），当前配置为5。例如下图短边9个格子，角点数为8；应按实际使用的标定板填写。

​		L :  标定板单个方格的实际边长（单位：米），当前配置为0.02726。请实测所用标定板，不要直接套用示例值。

​        ![](picture/image-20241016181226851.png)



#### 眼在手上

##### 采集数据

(1).相机连接线连接**电脑**和D435相机，网线连接**电脑**和机械臂

(2).电脑的ip和机械臂设置为同一网段

如果机械臂IP为192.168.1.18 ，则将电脑 ip地址设为1网段

![image-20241018143659263](picture/image-20241018143659263.png)

​			如果机械臂的IP为192.168.10.18，则将电脑的IP设置为10网段（参照上面设置）                                     

(3).标定板放置在平面上，相机固定在机械臂末端，相机对准标定板

​		标定过程中，标定板是**固定放置**在机械臂工作区内的，这个固定位置需要保证安装在**机械臂末端的相机能从不同的视角**观测到它，标定板的确切位置不重要，因为不必知道其相对于机械臂基座的位姿。但是标定板在标定期间应**保持固定，不得移动**。

(4).运行脚本`collect_data.py`，出现一个弹窗

(5).拖动机械臂末端，使呈现相机视野下的标定板清晰、完整、将光标放在弹窗上

​		**注意：**

- 使出现在相机视野里的**标定板**和**相机镜面**呈现**一定角度**。

  ![image-20241101112801145](picture/image-20241101112801145.png)

  下面的拍照姿势**不对**：
  ![image-20241101112816914](picture/image-20241101112816914.png)

​				

(6).点击键盘“s”采集数据

(7)移动15-20次机械臂，重复步骤(5)(6)，采集不同机械臂姿态下的标定板图片15-20张左右

​	**注意：**

​			移动**机械臂末端旋转轴**，使样本之间包含充分的旋转变化；30°可作为采样参考，不要求相邻两次都超过30°。

​			确保在三个轴（X、Y、Z）上都有足够的旋转角度变化。

​			（可以先绕着机械臂末端z轴旋转多个角度拍摄多副图片，然后绕着x轴旋转）

​			![WPS拼图1](picture/WPS拼图1.png)

##### 计算标定结果

​		运行脚本`compute_in_hand.py`，获取标定结果

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

(5).拖动机械臂末端，使呈现相机视野下的标定板清晰、完整、将光标放在弹窗上

​		**注意：**

​				使出现在相机视野里的标定板和相机镜面呈现一定角度。

(6).点击键盘“s”采集数据

(7)移动15-20次机械臂，重复步骤(5)(6)，采集不同机械臂姿态下的标定板图片15-20张左右

​			移动**机械臂末端旋转轴**，使样本之间包含充分的旋转变化；30°可作为采样参考，不要求相邻两次都超过30°。

​			确保在三个轴（X、Y、Z）上都有足够的旋转角度变化。



##### 计算标定结果

运行脚本`compute_to_hand.py`，获取标定结果

得出**相机坐标系**相对于**机械臂基坐标系**的**旋转矩阵**和**平移向量**



**平移向量**的单位是米



#### 误差范围

实际精度受图片质量、位姿配对、棋盘尺寸、安装稳定性和机械臂位姿精度等因素影响，不能保证平移误差在1厘米内。当前配置的12毫米限值检查的是同批数据的固定关系位置残差，不是实际定位误差；通过检查后仍需使用未参与标定的实测点验证。

## 标定过程中可能出现的问题

以下列出采集和计算阶段的常见问题。两个计算脚本 `compute_in_hand.py`、`compute_to_hand.py` 共用同一套计算检查；请先根据终端提示排查，再查看本批数据的结果文件。

### 问题1：有效旋转不足或旋转方向过于单一

**当前提示：**

```text
标定失败：有效旋转变化不足，请增加不同方向的旋转后重新采集
标定失败：旋转方向过于单一，请增加绕不同轴的姿态变化
```

当前程序会在调用 OpenCV 前检查有效相对旋转及旋转方向。现在这类数据通常会先触发上述中文提示，旋转不足的问题本身仍可能发生。

**处理方法：** 采集绕不同轴旋转的15～20组姿态，每次停稳后再按 `s`，并保证棋盘清晰、完整。避免只平移、只绕同一轴旋转，或大量重复近似姿态。30度只是采样建议，不是程序要求每次必须超过的固定门槛，旋转也不是越大越好；增加图片数量不能代替有效的旋转变化。

### 问题2：相机无法启动、按 s 没反应或样本未保存

**常见表现：** 没有相机预览窗口，或出现“采集失败，未保存本组数据”“机械臂通信失败，请检查连接并重新启动采集”“切换 Base 工作坐标系失败，停止采集”等提示。

**处理方法：**

- 相机无法启动时，检查相机连接、`pyrealsense2` 环境以及是否有其他程序占用相机。当前采集程序使用 D435 彩色流，分辨率640×480、30 FPS。
- 按 `s` 前先点击 `Capture_Video` 窗口，使其获得焦点，不要在终端输入。
- 检查机械臂网络、8080端口和控制器报错。当前程序依次尝试 `192.168.1.18`、`192.168.10.18`；其他IP需对应调整 `libs/auxiliary.py` 中的地址。
- 通信失败后连接会被关闭，应排除故障并重新启动采集。保存失败时检查目录权限和磁盘空间；未保存成功的样本不计数。

### 问题3：数据目录、图片与位姿配对不正确

**常见提示：** “未找到采集数据”“数据目录不存在”“图片编号重复”“无法可靠配对”，或 `poses.txt` 某行不是六个有限数值。

**处理方法：** 明确指定要计算的数据目录，检查其中的数字编号 `.jpg` 图片和 `poses.txt`。图片按数字顺序与位姿逐行配对，数量必须一致；不要单独删除图片、改编号、删位姿行或混用不同批次的数据。数量相同也不能证明配对正确，程序无法识别手工整理后顺序已错乱的数据；无法恢复原始对应关系时应重新采集。

### 问题4：棋盘角点检测失败、图片被剔除或有效配对不足

**常见提示：** “未检测到完整棋盘角点”“重投影误差过大”“标定板位姿无效”，以及“有效配对仅……组”或“质量检查后仅剩……组有效配对”。

**处理方法：**

- 检查 `config.yaml` 的 `XX`、`YY` 是否与实际棋盘的内角点数一致，不是方格数。
- 保证棋盘完整入镜、角点清晰，避免模糊、反光、遮挡和过度倾斜；停稳后采样。
- 当前单图重投影均方根误差限值为2像素。角点、位姿或重投影检查未通过的图片会连同对应机械臂位姿一起剔除；剔除后至少保留6组，建议重新补采至15～20组不同姿态。

### 问题5：棋盘参数、相机内参或畸变模型不匹配

**常见提示：** “棋盘内角点数必须为大于1的整数，格长必须为正数，单位为米”“相机内参无效或与图片分辨率不匹配”，或畸变模型不能直接作为 OpenCV Brown 参数使用。

**处理方法：** 使用实际棋盘的内角点数和实测格长 `L`，单位为米；打印缩放后应重新测量。使用本批采集生成的 `camera.json` 和原始分辨率图片，不要复制其他相机的内参，或对图片缩放、裁剪后继续使用原内参。当前固定内参路径支持 Brown 畸变参数及无畸变图像，不支持的非零畸变模型需先做相应适配。

旧数据缺少 `camera.json` 时，程序会提示并从本批图片估计内参，可以继续计算，但仍需检查结果质量，不能随意补入其他批次的内参文件。

### 问题6：求解结果无效或固定关系误差超限

**当前提示：**

```text
标定失败：手眼求解返回无效变换，请检查采样姿态和数据配对
标定失败：固定关系误差超限，结果已标记 rejected；请检查配对、棋盘参数和采样质量
```

**处理方法：** 检查图片与位姿是否对应、棋盘参数是否准确、计算入口是否匹配安装方式，以及采样过程中固定关系是否改变。眼在手上时，标定板与基座、相机与工具的关系应保持不变；眼在手外时，相机与基座、标定板与工具的关系应保持不变。采集期间不要切换工具或工作坐标系。

当前配置要求固定关系位置最大偏差不超过12毫米、姿态最大偏差不超过3度，以实际 `config.yaml` 为准。超限结果保存为 `rejected`；可查看结果文件的 `quality`、`limits`、`accepted_images` 和 `rejected_images` 定位问题。应先检查采样和配对原因，不要仅为通过检查而放宽限值。

### 问题7：结果为 candidate，但实际定位仍有偏差

`candidate` 只表示通过本批数据的检查，不保证实际定位精度。以下情况目前不能由程序完全自动识别：

| 可能原因 | 当前限制及处理方法 |
| --- | --- |
| 运动中采样，图像与位姿不同步 | 当前程序取图后再查询机械臂位姿，没有自动停稳检测或按时间戳配对。应停稳后按 `s`，保持静止至本组保存完成。 |
| 安装松动或中途切换坐标系 | 程序不会持续检测安装状态或工具、工作坐标系变化。检查并固定安装，保持坐标系一致，变更后重新采集。 |
| 格长或内参错误但格式合法 | 合法数值、相同分辨率不代表参数对应实际设备。实测格长，使用本相机、本批次的内参。 |
| 图片与位姿数量相同但顺序错误 | 数量和格式检查无法确认实际时序。保留原始采集顺序，无法恢复对应关系时重新采集。 |
| 使用了错误的变换方向或单位 | 眼在手上输出 `camera_to_end`，眼在手外输出 `camera_to_base`；平移单位为米，四元数顺序为 `[x,y,z,w]`。使用结果时核对这些字段。 |
|臂本身的定位精度较低|若是连续3-4次算出来的结果位置残差的最大值均大于10mm，建议更换臂、激光标定仪精标等等。个人建议位置残差的MAX<5mm/RMS<3mm|

即使重投影误差和固定关系残差较小，也应使用未参与标定的实测点验证实际精度。计算已开始且结果文件可写时，提前失败会将状态记为 `failed`，误差超限为 `rejected`，通过检查为 `candidate`；不要把保存了矩阵或打印了数值当作标定已经验证成功。

## 手眼标定结果如何使用



机械臂如何捡取物品？

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

$$ H^{ROB}_{OBJ} = H^{ROB}_{EE} \cdot H^{EE}_{CAM} \cdot H^{CAM}_{OBJ} $$

末端执行器相对于机械臂基座的位姿

$$H^{ROB}_{EE}$$

是已知的，通过机械臂API可以获取得到，相机相对于末端执行器的位姿

$$H^{EE}_{CAM}$$

由手眼标定得到。

![img](picture/hand-eye-eye-in-hand-all-poses.png)



假如物体在相机坐标系是一个3D点或者一个位姿，那么下面说明物体位置作为一个3D点或者一个位姿**从相机坐标系转换到机械臂基坐标系**的数学理论。

![../../../_images/hand-eye-eye-in-hand-all-poses.png](picture/hand-eye-eye-in-hand-all-poses-17302010389227.png)

机械臂的位姿用齐次变换矩阵表示。

以下方程描述了如何将单个3D点从相机坐标系转换到机械臂基坐标系：

$$ \widetilde{p}^{ROB} = H^{ROB}_{EE} \cdot H^{EE}_{CAM} \cdot \widetilde{p}^{CAM} $$

$$ \begin{bmatrix} x^r \\ y^r \\ z^r \\ 1 \end{bmatrix} = \begin{bmatrix} R_e^r & t_e^r \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} R_c^e & t_c^e \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} x^c \\ y^c \\ z^c \\ 1 \end{bmatrix} $$

如果要将物体位姿从相机坐标系转换到机械臂基坐标系。

$$ H^{ROB}_{OBJ} = H^{ROB}_{EE} \cdot H^{EE}_{CAM} \cdot H^{CAM}_{OBJ} $$

$$ \begin{bmatrix} R_o^r & t_o^r \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} R_e^r & t_e^r \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} R_c^e & t_c^e \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} R_o^c & t_o^c \\ 0 & 1 \end{bmatrix} $$


​	由此产生的位姿是机械臂当前工具坐标系圆心应该达到的位姿进行捡取。(标定时采集的机械臂位姿也是当前工具坐标系相对于机械臂基坐标系位姿)



#### 代码



- 物体在相机坐标系是一个3D点（x,y,z)

  ```python
  
  import numpy as np
  from scipy.spatial.transform import Rotation as R
  
  
  # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量（手眼标定得到）
  rotation_matrix = np.array([[-0.00235395 , 0.99988123 ,-0.01523124],
                              [-0.99998543, -0.00227965, 0.0048937],
                              [0.00485839, 0.01524254, 0.99987202]])
  translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])
  
  
  def convert(x ,y ,z ,x1 ,y1 ,z1 ,rx ,ry ,rz):
      """
      我们需要将手眼标定得到旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
      机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体相对于机械臂基座的位姿（x, y, z）
  
      """
  
  
  
      # 深度相机识别物体返回的坐标
      obj_camera_coordinates = np.array([x, y, z])
  
      # 机械臂末端的位姿，单位为弧度
      end_effector_pose = np.array([x1, y1, z1,
                                    rx, ry, rz])
  
      # 将旋转矩阵和平移向量转换为齐次变换矩阵
      T_camera_to_end_effector = np.eye(4)
      T_camera_to_end_effector[:3, :3] = rotation_matrix
      T_camera_to_end_effector[:3, 3] = translation_vector
  
      # 机械臂末端的位姿转换为齐次变换矩阵
      position = end_effector_pose[:3]
      orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
  
      T_base_to_end_effector = np.eye(4)
      T_base_to_end_effector[:3, :3] = orientation
      T_base_to_end_effector[:3, 3] = position
  
      # 计算物体相对于机械臂基座的位姿
      obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标
  
      obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
  
      obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
  
      obj_base_coordinates = list(obj_base_coordinates_homo[:3])  # 从齐次坐标中提取物体的x, y, z坐标
  
  
      return obj_base_coordinates
  
  
  ```

  

- 物体在相机坐标系是一个位姿

  ```python
  
  import numpy as np
  from scipy.spatial.transform import Rotation as R
  
  
  # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量（手眼标定得到）
  rotation_matrix = np.array([[-0.00235395 , 0.99988123 ,-0.01523124],
                              [-0.99998543, -0.00227965, 0.0048937],
                              [0.00485839, 0.01524254, 0.99987202]])
  translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])
  
  
  def decompose_transform(matrix):
      """
      将矩阵转化为位姿
      """
  
      translation = matrix[:3, 3]
      rotation = matrix[:3, :3]
  
      # Convert rotation matrix to euler angles (rx, ry, rz)
      sy = np.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
      singular = sy < 1e-6
  
      if not singular:
          rx = np.arctan2(rotation[2, 1], rotation[2, 2])
          ry = np.arctan2(-rotation[2, 0], sy)
          rz = np.arctan2(rotation[1, 0], rotation[0, 0])
      else:
          rx = np.arctan2(-rotation[1, 2], rotation[1, 1])
          ry = np.arctan2(-rotation[2, 0], sy)
          rz = 0
  
      return translation, rx, ry, rz
  
  
  def convert(x,y,z,rx,ry,rz,x1,y1,z1,rx1,ry1,rz1):
  
      """
  
      我们需要将标定得到的旋转向量和平移向量转换为齐次变换矩阵，然后使用机械臂末端的位姿(x, y, z,rx,ry,rz）和
     深度相机识别到的物体坐标（x1,y1,z1,rx1,ry1,rz1）来计算物体相对于机械臂基座的位姿
  
      """
  
  
  
      # 深度相机识别物体返回的坐标
      obj_camera_coordinates = np.array([x1, y1, z1,rx1,ry1,rz1])
  
      # 机械臂末端的位姿，单位为弧度
      end_effector_pose = np.array([x, y, z,
                                    rx, ry, rz])
  
      # 将旋转矩阵和平移向量转换为齐次变换矩阵
      T_camera_to_end_effector = np.eye(4)
      T_camera_to_end_effector[:3, :3] = rotation_matrix
      T_camera_to_end_effector[:3, 3] = translation_vector
  
      # 机械臂末端的位姿转换为齐次变换矩阵
      position = end_effector_pose[:3]
      orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
  
      T_end_to_base_effector = np.eye(4)
      T_end_to_base_effector[:3, :3] = orientation
      T_end_to_base_effector[:3, 3] = position
  
      # 计算物体相对于机械臂基座的位姿
  
  
      # 物体相对于相机的位姿转换为齐次变换矩阵
      position2 = obj_camera_coordinates[:3]
      orientation2 = R.from_euler('xyz', obj_camera_coordinates[3:], degrees=False).as_matrix()
  
      T_object_to_camera_effector = np.eye(4)
      T_object_to_camera_effector[:3, :3] = orientation2
      T_object_to_camera_effector[:3, 3] = position2
  
  
      obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(T_object_to_camera_effector)
  
      obj_base_effector = T_end_to_base_effector.dot(obj_end_effector_coordinates_homo)
  
      result = decompose_transform(obj_base_effector)
  
  
  
      return result
  
  
  
  ```
  
  

​		   代码中rotation_matrix和translation_vector变量分别是眼在手上**手眼标定**得到的**旋转矩阵**和**平移向量**

### 眼在手外

#### 理论

相机可以通过模型获取物体在相机坐标系的里的位姿，**物体相对于机械臂的位姿**通过相机相对于机械臂基坐标系的位姿和物体相对于相机相机坐标系的位姿通过后乘法计算得到的：

$$ H^{ROB}_{OBJ} = H^{ROB}_{CAM} \cdot H^{CAM}_{OBJ} $$

![../../../_images/hand-eye-eye-to-hand-all-poses.png](picture/hand-eye-eye-to-hand-all-poses.png)



假如物体位置是一个3D点或者一个位姿，那么下面说明物体位置作为一个3D点或者一个位姿**从相机坐标系转换到机械臂基坐标系**的数学理论。

以下方程描述了如何将单个3D点从相机坐标系转换到机械臂基坐标系：

$$ \widetilde{p}^{ROB} = H^{ROB}_{CAM} \cdot \widetilde{p}^{CAM} $$

$$ \begin{bmatrix} x^r \\ y^r \\ z^r \\ 1 \end{bmatrix} = \begin{bmatrix} R_c^r & t_c^r \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} x^c \\ y^c \\ z^c \\ 1 \end{bmatrix} $$

如果要将物体位姿从相机坐标系转换到机械臂基坐标系。

$$ H^{ROB}_{OBJ} = H^{ROB}_{CAM} \cdot H^{CAM}_{OBJ} $$

$$ \begin{bmatrix} R_o^r & t_o^r \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} R_c^r & t_c^r \\ 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} R_o^c & t_o^c \\ 0 & 1 \end{bmatrix} $$

#### 代码

- 物体在相机坐标系是一个3D点（x,y,z)

  ```python
  
  import numpy as np
  
  
  
  # 相机坐标系到机械臂基坐标系的旋转矩阵和平移向量(手眼标定得到)
  rotation_matrix = np.array([[-0.00235395 , 0.99988123 ,-0.01523124],
                              [-0.99998543, -0.00227965, 0.0048937],
                              [0.00485839, 0.01524254, 0.99987202]])
  translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])
  
  def convert(x ,y ,z):
      """
      我们需要将旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
      标定好的 相机到基座的 齐次变换矩阵 来计算物体相对于机械臂基座的位姿（x, y, z）
  
      """
  
  
  
      # 深度相机识别物体返回的坐标
      obj_camera_coordinates = np.array([x, y, z])
  
  
      # 将旋转矩阵和平移向量转换为齐次变换矩阵
      T_camera_to_base_effector = np.eye(4)
      T_camera_to_base_effector[:3, :3] = rotation_matrix
      T_camera_to_base_effector[:3, 3] = translation_vector
  
  
  
      # 计算物体相对于机械臂基座的位姿
      obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标
  
      obj_base_effector_coordinates_homo = T_camera_to_base_effector.dot(obj_camera_coordinates_homo)
      obj_base_coordinates = obj_base_effector_coordinates_homo[:3]  # 从齐次坐标中提取物体的x, y, z坐标
  
  
      # 组合结果
  
      return list(obj_base_coordinates)
  
  
  
  ```

  

- 物体在相机坐标系是一个位姿

  ```python
  
  import numpy as np
  from scipy.spatial.transform import Rotation as R
  
  
  # 相机坐标系到机械臂基坐标系的旋转矩阵和平移向量（手眼标定得到）
  rotation_matrix = np.array([[-0.00235395 , 0.99988123 ,-0.01523124],
                              [-0.99998543, -0.00227965, 0.0048937],
                              [0.00485839, 0.01524254, 0.99987202]])
  translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])
  
  def decompose_transform(matrix):
      """
      将矩阵转化为位姿
      """
  
      translation = matrix[:3, 3]
      rotation = matrix[:3, :3]
  
      # Convert rotation matrix to euler angles (rx, ry, rz)
      sy = np.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
      singular = sy < 1e-6
  
      if not singular:
          rx = np.arctan2(rotation[2, 1], rotation[2, 2])
          ry = np.arctan2(-rotation[2, 0], sy)
          rz = np.arctan2(rotation[1, 0], rotation[0, 0])
      else:
          rx = np.arctan2(-rotation[1, 2], rotation[1, 1])
          ry = np.arctan2(-rotation[2, 0], sy)
          rz = 0
  
      return translation, rx, ry, rz
  
  
  
  def convert(x ,y ,z,rx,ry,rz):
      """
      我们需要将旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体位姿（x, y, z,rx,ry,rz）和
      标定好的 相机到基座的 齐次变换矩阵 来计算物体相对于机械臂基座的位姿（x, y, z,rx,ry,rz）
  
      """
  
  
  
      # 深度相机识别物体返回的坐标
      obj_camera_coordinates = np.array([x, y, z,rx,ry,rz])
  
  
      # 将旋转矩阵和平移向量转换为齐次变换矩阵
      T_camera_to_base_effector = np.eye(4)
      T_camera_to_base_effector[:3, :3] = rotation_matrix
      T_camera_to_base_effector[:3, 3] = translation_vector
  
  
  
      # 计算物体相对于机械臂基座的位姿
      # 物体相对于相机的位姿转换为齐次变换矩阵
      position2 = obj_camera_coordinates[:3]
      orientation2 = R.from_euler('xyz', obj_camera_coordinates[3:], degrees=False).as_matrix()
  
      T_object_to_camera_effector = np.eye(4)
      T_object_to_camera_effector[:3, :3] = orientation2
      T_object_to_camera_effector[:3, 3] = position2
  
      obj_base_effector = T_camera_to_base_effector.dot(T_object_to_camera_effector)
  
  	result = decompose_transform(obj_base_effector)
  
      # 组合结果
  
      return result
  
  ```

​		代码中rotation_matrix和translation_vector变量分别是眼在手外**手眼标定**得到的**旋转矩阵**和**平移向量**

### 
