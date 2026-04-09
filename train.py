from ultralytics import YOLO

model = YOLO('yolov8n.pt')

model.train(
    data='/Users/bhaskarsingh/Desktop/UNI/3rd year/winter/ece342/project/onPC/342.yolov8/data.yaml',
    epochs=50,      
    imgsz=640,
    batch=8,
    name='cube_bowl_detector',
    split='train'
)

metrics = model.val(
    data='/Users/bhaskarsingh/Desktop/UNI/3rd year/winter/ece342/project/onPC/342.yolov8/data.yaml',
    split='test'
)

