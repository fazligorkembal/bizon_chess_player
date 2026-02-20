from ultralytics import YOLO

model = YOLO("yolo26x-cls.pt")
model.export(format="onnx", dynamic=False, simplify=True, opset=16)