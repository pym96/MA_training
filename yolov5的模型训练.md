# YOLOv5 模型训练与导出

## 数据集

数据集：华农开源数据集

[SCAU-RM-NAV/RM2023_Radar_Dataset: 该仓库为RM2023雷达站所用到的yolo神经网络训练数据集，包含车和装甲板（上交格式）。同时有yolov5 6.0的训练环境，可以在本地进行训练和测试 (github.com)](https://github.com/SCAU-RM-NAV/RM2023_Radar_Dataset)

训练集 `train`----------学生的课本；学生 根据课本里的内容来掌握知识。

验证集 `val`------------作业，通过作业可以知道 不同学生学习情况、进步的速度快慢。

测试集 `test`（可选）-----------考试，考的题是平常都没有见过，考察学生举一反三的能力。

## 模型训练

[ultralytics/yolov5: YOLOv5 🚀 in PyTorch > ONNX > CoreML > TFLite (github.com)](https://github.com/ultralytics/yolov5)

```git
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
```

## 安装依赖

```python
pip install -r requirements.txt
```

## 将数据集RMdatasets移到yolov5文件夹

```
username:~/Coding/yolov5$ tree -d
.
├── classify
├── data
│   ├── hyps
│   ├── images
│   └── scripts
├── models
│   ├── hub
│   └── segment
├── RMdatasets
│   └── car
│       ├── train
│       │   ├── images
│       │   └── labels
│       └── val
│           ├── images
│           └── labels
├── segment
└── utils
    ├── aws
    ├── docker
    ├── flask_rest_api
    ├── google_app_engine
    ├── loggers
    │   ├── clearml
    │   ├── comet
    │   └── wandb
    └── segment
```

将 `data` 文件夹中的 `coco128.yaml` 再复制一份到 `data` 里，重命名为 `RM_car.yaml` ，内容为：

```
# Train/val/test sets as 1) dir: path/to/imgs, 2) file: path/to/imgs.txt, or 3) list: [path/to/imgs1, path/to/imgs2, ..]

path: RMdatasets/car/  # dataset root dir
train: train/images  # train images (relative to 'path') 
val: val/images  # val images (relative to 'path') 
test:  # test images (optional)

# Classes
nc: 1
names:
  0: car
```

指定权重文件yolov5s.pt,会自动在Github中的yolov5中找到yolov5s.pt

## 开始训练

```
python3 train.py --img 640 --batch 32 --epochs 150 --data ./data/RM_car.yaml --cfg ./models/yolov5s.yaml --weights ./weight/yolov5s.pt --device 0
```

