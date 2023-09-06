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

## 将数据集RMdatasets移到yolov5文件夹（这里仅以 car 为示范）

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

```yaml
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

## 将 data 文件夹中的 coco128.yaml 再复制一份到 data 里，重命名为 RM_armor.yaml 

```yaml
# Train/val/test sets as 1) dir: path/to/imgs, 2) file: path/to/imgs.txt, or 3) list: [path/to/imgs1, path/to/imgs2, ..]

path: RMdatasets/armor/  # dataset root dir
train: train/images  # train images (relative to 'path') 
val: val/images  # val images (relative to 'path') 
test:  # test images (optional)

# Classes
nc: 12
names:
  0: B1
  1: B2
  2: B3
  3: B4
  4: B5
  5: B_AI
  6: R1
  7: R2
  8: R3
  9: R4
  10: R5
  11: R_AI
```

指定权重文件为 yolov5s.pt ,会自动在 Github 中的 yolov5 里找到 yolov5s.pt

## 开始训练

```python
python3 train.py --img 640 --batch 32 --epochs 150 --data ./data/RM_armor.yaml --cfg ./models/yolov5s.yaml --weights ./weight/yolov5s.pt --device 0
```

## 导出为 .onnx 模型

```
python3 export.py --weights ./weight/RMArmor_clh829.pt --include onnx --dynamic 
// -- dynamic 表示导出动态模型
```

```
python3 export.py --weights ./weight/RMCar_clh829.pt --include onnx --dynamic
// -- dynamic 表示导出动态模型
```

## 模型推理

### Car

```
python3 detect.py --weights /home/lihanchen/Coding/yolov5/weight/RMCar_clh829.pt --source /home/lihanchen/Coding/RM_Radar2023/resources/RadarVideo.mp4
```

### Armour

```
python3 detect.py --weights /home/lihanchen/Coding/yolov5/weight/RMArmor_clh829.pt --source /home/lihanchen/Coding/RM_Radar2023/resources/RadarVideo.mp4
```

```
python3 detect.py --weights /home/lihanchen/Coding/yolov5/weight/RMArmor_clh829.pt --source /home/lihanchen/Coding/yolov5/RMdatasets/car/train/images/002987.jpg
```

## test 推理onnx

```
pip install onnxruntime-gpu
```

```
python3 detect.py --weights /home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/43best.onnx --source /home/lihanchen/Coding/RM_Radar2023/resources/RadarVideo.mp4
```

```
#define OnnxMoudlePath (char *)"/home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/RMArmor_clh829.onnx"
#define OnnxMoudlePath_c (char *)"/home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/RMCar_clh829.onnx"
#define TensorRTEnginePath (char *)"/home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/RMArmor_clh829.engine"     // Engine
#define TensorRTEnginePath_c (char *)"/home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/RMCar_clh829.engine" // Engine
```

onnx2trt_utils.cpp:377: Your ONNX model has been generated with INT64 weights, while TensorRT does not natively support INT64. Attempting to cast down to INT32. 
TensorRT encountered issues when converting weights between types and that could affect accuracy. 
If this is not the desired behavior, please modify the weights or retrain with regularization to adjust the magnitude of the weights. Check verbose logs for the list of affected weights. 
- 46 weights are affected by this issue: Detected subnormal FP16 values. 
- 3 weights are affected by this issue: Detected values less than smallest positive FP16 subnormal value and converted them to the FP16 minimum subnormalized value.

```
#define OnnxMoudlePath (char *)"/home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/43best.onnx"
#define OnnxMoudlePath_c (char *)"/home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/best.onnx"
#define TensorRTEnginePath (char *)"/home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/model_trt.engine"     // Engine
#define TensorRTEnginePath_c (char *)"/home/lihanchen/Coding/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/model_trt_c.engine" // Engine
```

onnx2trt_utils.cpp:377: Your ONNX model has been generated with INT64 weights, while TensorRT does not natively support INT64. Attempting to cast down to INT32.
TensorRT encountered issues when converting weights between types and that could affect accuracy.
If this is not the desired behavior, please modify the weights or retrain with regularization to adjust the magnitude of the weights.
Check verbose logs for the list of affected weights.

- 51 weights are affected by this issue: Detected subnormal FP16 values.
TensorRT encountered issues when converting weights between types and that could affect accuracy.
If this is not the desired behavior, please modify the weights or retrain with regularization to adjust the magnitude of the weights.
Check verbose logs for the list of affected weights.

- 51 weights are affected by this issue: Detected subnormal FP16 values.

- 1 weights are affected by this issue: Detected values less than smallest positive FP16 subnormal value and converted them to the FP16 minimum subnormalized value.

```
map<int, int> _ids = {{0, 6}, {1, 7}, {2, 8}, {3, 9}, {4, 10}, {5, 11}, {6, 0}, {7, 1}, {8, 2}, {9, 3}, {10, 4}, {11, 5}};
```

