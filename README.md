# HuaWei_Atlas_Gesture_Recognition
## 项目介绍
本项目主要为在华为Atlas200DK开发板上部署OpenPose模型以及动作识别模型。通过OpenPose识别人体骨架关键点的位置信息，并将识别结果组成序列发送到动作识别Engine中分辩人体的动作，以达到机器人根据人的动作做出相应的动作。
### 开发环境
MindStudio1.32版
### Graph结构
DataInput（读取图片数据）->ImagePreProcess（图片预处理）->MindInferenceEngine（OpenPose识别引擎）->SaveFilePostProcess（识别结果处理）->MindInferenceEngine_pose（动作识别引擎处理动作识别结果，与机器人通信）
## 使用教程
 * 下载模型:[模型下载链接](
https://drive.google.com/drive/folders/1PlTvGIgjrBD1w8bT5bzsIki-xEjjn4R1?usp=sharing)

其中pose_deploy.om是用于OpenPose识别骨架的，stgcn_fps50_sta_ho_ki.om是用于识别骨架动作序列的
 * 修改src/graph.config文件，主要修改模型路径以及输入图像的路径
 * 按照MindStudio教程部署到Atlas 200DK中，具体请参考[MindStudio手册](https://www.huaweicloud.com/ascend/doc/mindstudio/2.1.0(beta)/zh/zh-cn_topic_0200347877.html)
 * 修改send_image/send_image.py文件，可以选择识别本地的视频文件或者实时的摄像头文件
 * 终端ssh连接Atlas，运行openpose_det项目，PC运行send_image.py，即可看到实时的识别效果
 * 
 * 

