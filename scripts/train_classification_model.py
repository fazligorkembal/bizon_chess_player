from ultralytics import YOLO

# Load a model
model = YOLO("yolo26m-cls.pt")

# Train the model with MPS
results = model.train(data="/home/user/Documents/bizon_chess_player/datasets/box_classification", epochs=5, device="cuda", workers=12, batch=32)