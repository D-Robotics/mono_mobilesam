[English](./README.md) | 简体中文

Getting Started with mono mobilesam
=======

# 功能介绍

mono_mobilesam package是基于 Mobile SAM 量化部署的使用示例。图像数据来源于本地图片回灌和订阅到的image msg。SAM 依赖检测框输入进行分割, 并分割检测框中的目标, 无需指定目标的类别信息, 仅需提供框。

本示例中, 我们提供了两种部署展示方式:
- 固定框分割：固定了检测框（图片中央）用以分割。
- 订阅框分割：订阅上游检测网络输出的检测框信息, 对框中的信息进行分割。

# 开发环境

- 编程语言: C/C++
- 开发平台: X5/X86
- 系统版本：Ubuntu 22.04
- 编译工具链: Linux GCC 11.4.0

# 编译

- X5版本：支持在X5 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

- X86版本：支持在X86 Ubuntu系统上编译一种方式。

同时支持通过编译选项控制编译pkg的依赖和pkg的功能。

## 依赖库

- opencv:3.4.5

ros package：

- dnn node
- cv_bridge
- sensor_msgs
- hbm_img_msgs
- ai_msgs

hbm_img_msgs为自定义的图片消息格式, 用于shared mem场景下的图片传输, hbm_img_msgs pkg定义在hobot_msgs中, 因此如果使用shared mem进行图片传输, 需要依赖此pkg。


## 编译选项

1、SHARED_MEM

- shared mem（共享内存传输）使能开关, 默认打开（ON）, 编译时使用-DSHARED_MEM=OFF命令关闭。
- 如果打开, 编译和运行会依赖hbm_img_msgs pkg, 并且需要使用tros进行编译。
- 如果关闭, 编译和运行不依赖hbm_img_msgs pkg, 支持使用原生ros和tros进行编译。
- 对于shared mem通信方式, 当前只支持订阅nv12格式图片。

## X5 Ubuntu系统上编译

1、编译环境确认

- 板端已安装X5 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon, 需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select mono_mobilesam`

## docker交叉编译 X5版本

1、编译环境确认

- 在docker中编译, 并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译dnn node package
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令：

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s mono_mobilesam
  ```

- 编译选项中默认打开了shared mem通信方式。


## X86 Ubuntu系统上编译 X86版本

1、编译环境确认

  x86 ubuntu版本: ubuntu22.04
  
2、编译

- 编译命令：

  ```shell
  colcon build --packages-select mono_mobilesam \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DPLATFORM_X86=ON \
     -DTHIRD_PARTY=`pwd`/../sysroot_docker
  ```

## 注意事项


# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- usb_cam package：发布图片msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名             | 解释                                  | 是否必须             | 数值类型 | 默认值                 |
| ------------------ | ------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| cache_len_limit          | 设置缓存的图片buffer长度            | 否                   | int | 8                   |                                                                         |
| feed_type          | 图片来源, 0：本地；1：订阅            | 否                   | int | 0                   |                                                                         |
| image              | 本地图片地址                          | 否                   | string | config/00131.jpg     |                                                                         |
| is_shared_mem_sub  | 使用shared mem通信方式订阅图片        | 否                   | int | 0                   |                                                                         |
| is_regular_box  | 使用固定检测框输入SAM        | 否                   | int | 0                   |                                                                         |
| dump_render_img    | 是否进行渲染，0：否；1：是            | 否                   | int | 0                   |                                                                         |
| ai_msg_sub_topic_name | 订阅上游检测结果的topicname,用于SAM输入 | 否                   | string | /hobot_dnn_detection | |
| ai_msg_pub_topic_name | 发布智能结果的topicname,用于web端展示 | 否                   | string | /hobot_sam | |
| ros_img_sub_topic_name | 接收ros图片话题名 | 否                   | string | /image | |

## 使用说明

- 控制话题：mono_mobilesam 支持通过ai msg话题消息获取目标检测框。使用示例：
```shell
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 96, "y_offset": 96, "width": 192, "height": 96}, "type": "anything"}]}] }'
```

## 运行

- mono_mobilesam 使用到的模型在安装包'config'路径下。

- 编译成功后, 将生成的install路径拷贝到地平线RDK上（如果是在RDK上编译, 忽略拷贝步骤）, 并执行如下命令运行。

## X5 Ubuntu系统上运行

运行方式1, 使用可执行文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash
# config中为示例使用的模型, 回灌使用的本地图片
# 根据实际安装路径进行拷贝（docker中的安装路径为install/lib/mono_mobilesam/config/, 拷贝命令为cp -r install/lib/mono_mobilesam/config/ .）。
cp -r install/lib/mono_mobilesam/config/ .

# 运行模式1：
# 使用本地jpg格式图片进行回灌预测
ros2 run mono_mobilesam mono_mobilesam --ros-args -p feed_type:=0 -p image:=config/00131.jpg -p image_type:=0 -p dump_render_img:=1

# 运行模式2：使用shared mem通信方式(topic为/hbmem_img)进行预测,使用固定检测框进行SAM检测
ros2 run mono_mobilesam mono_mobilesam --ros-args -p feed_type:=1 -p is_shared_mem_sub:=1 -p is_regular_box:=1 --ros-args --log-level warn

# 运行模式3：
# 使用shared mem通信方式(topic为/hbmem_img)进行预测, 设置ai订阅话题名(/hobot_dnn_detection)为并设置log级别为warn。同时在另一个窗口发送ai msg话题(topic为/hobot_dnn_detection) 变更检测框
ros2 run mono_mobilesam mono_mobilesam --ros-args -p feed_type:=1 --ros-args --log-level warn -p ai_msg_sub_topic_name:="/hobot_dnn_detection"

ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 96, "y_offset": 96, "width": 192, "height": 96}, "type": "anything"}]}] }'

```

运行方式2, 使用launch文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型, 根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项）, 拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., 其中PKG_NAME为具体的package名。
cp -r install/lib/mono_mobilesam/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 运行模式1：启动launch文件, 单独启动 sam 节点
ros2 launch mono_mobilesam sam.launch.py

# 运行模式2：启动launch文件, 启动检测节点 + sam节点
ros2 launch mono_mobilesam sam_with_yolo_world.launch.py
```

## X5 yocto系统上运行

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型, 回灌使用的本地图片
cp -r install/lib/mono_mobilesam/config/ .

# 运行模式1：
# 使用本地jpg格式图片进行回灌预测, 输入自定义类别
./install/lib/mono_mobilesam/mono_mobilesam --ros-args -p feed_type:=0 -p image:=config/00131.jpg -p image_type:=0 -p texts:="liquid stain,mild stain,solid stain,congee stain" -p dump_render_img:=1

# 运行模式2：使用shared mem通信方式(topic为/hbmem_img)进行预测,使用固定检测框进行SAM检测
./install/lib/mono_mobilesam/mono_mobilesam --ros-args -p feed_type:=1 -p is_shared_mem_sub:=1 -p is_regular_box:=1 --ros-args --log-level warn

# 运行模式3：
# 使用订阅到的image msg(topic为/image)进行预测, 设置ai订阅话题名(/hobot_dnn_detection)为并设置log级别为warn。同时在另一个窗口发送ai msg话题(topic为/hobot_dnn_detection) 变更检测框
./install/lib/mono_mobilesam/mono_mobilesam --ros-args -p feed_type:=1 --ros-args --log-level warn -p ai_msg_sub_topic_name:="/hobot_dnn_detection"

ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 96, "y_offset": 96, "width": 192, "height": 96}, "type": "anything"}]}] }'
```

## X86 Ubuntu系统上运行

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型, 根据实际安装路径进行拷贝
cp -r ./install/lib/mono_mobilesam/config/ .

export CAM_TYPE=fb

# 运行模式1：启动launch文件, 单独启动 sam 节点
ros2 launch mono_mobilesam sam.launch.py

# 运行模式2：启动launch文件, 启动检测节点 + sam节点
ros2 launch mono_mobilesam sam_with_yolo_world.launch.py
```

# 结果分析

## X5结果展示

log：

运行命令：`ros2 run mono_mobilesam mono_mobilesam --ros-args -p feed_type:=0 -p image:=config/00131.jpg -p dump_render_img:=1`

```shell
[WARN] [0000084147.330430207] [mono_mobilesam]: Parameter:
 cache_len_limit: 8
 dump_render_img: 1
 feed_type(0:local, 1:sub): 0
 image: config/00131.jpg
 is_regular_box: 0
 is_shared_mem_sub: 1
 is_sync_mode: 0
 ai_msg_pub_topic_name: /hobot_sam
 ai_msg_sub_topic_name: /hobot_yolo_world
 ros_img_sub_topic_name: /image
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.52.0
[DNN] Runtime version = 1.23.9_(3.15.52 HBRT)
[A][DNN][packed_model.cpp:247][Model](1970-01-01,23:22:27.721.321) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](1970-01-01,23:22:27.857.685) Model: mobilesam_encoder_384_all_BPU. Inconsistency between the hbrt library version 3.15.52.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[A][DNN][packed_model.cpp:247][Model](1970-01-01,23:22:28.71.135) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](1970-01-01,23:22:28.149.306) Model: mobilesam_decoder_384. Inconsistency between the hbrt library version 3.15.52.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[INFO] [0000084148.151637833] [mono_mobilesam]:
Model Info:
name: mobilesam_encoder_384_all_BPU.
[input]
 - (0) Layout: NCHW, Shape: [1, 3, 384, 384], Type: HB_DNN_IMG_TYPE_RGB.
[output]
 - (0) Layout: NCHW, Shape: [1, 256, 24, 24], Type: HB_DNN_TENSOR_TYPE_F32.

Model Info:
name: mobilesam_decoder_384.
[input]
 - (0) Layout: NCHW, Shape: [1, 5, 4, 1], Type: HB_DNN_TENSOR_TYPE_F32.
 - (1) Layout: NCHW, Shape: [1, 256, 24, 24], Type: HB_DNN_TENSOR_TYPE_F32.
[output]
 - (0) Layout: NCHW, Shape: [5, 4, 1, 1], Type: HB_DNN_TENSOR_TYPE_F32.
 - (1) Layout: NHWC, Shape: [5, 4, 96, 96], Type: HB_DNN_TENSOR_TYPE_F32.

[INFO] [0000084148.460530249] [MobileSam]: Draw result to file: render_feedback_0_0.jpeg
```

## 渲染结果
![image](img/render_sam_0.jpeg)

说明：前处理对图片进行缩放和补全处理。