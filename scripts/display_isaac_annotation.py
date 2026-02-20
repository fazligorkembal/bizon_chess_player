import numpy as np
import cv2

class_id_map = {
    0: "white_bishop",
    1: "white_rook",
    2: "white_knight",
    3: "white_queen",
    4: "white_king",
    5: "white_pawn",
    6: "black_bishop",
    7: "black_rook",
    8: "black_knight",
    9: "black_queen",
    10: "black_king",
    11: "black_pawn"
}


if __name__ == "__main__":
    for i in range(2000):
        label_path = f"/home/user/Documents/bizon_chess_player/datasets/chessboard_train/bounding_box_2d_tight_{i:04d}.npy"
        image_path = f"/home/user/Documents/bizon_chess_player/datasets/chessboard_train/rgb_{i:04d}.png"

        bboxes = np.load(label_path, allow_pickle=True)
        image = cv2.imread(image_path)
        try:
            for bbox in bboxes:
                class_id = int(bbox["semanticId"])
                x_min = int(bbox["x_min"])
                y_min = int(bbox["y_min"])
                x_max = int(bbox["x_max"])
                y_max = int(bbox["y_max"])
                cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(image, class_id_map[class_id], (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
            cv2.imshow("Image", image)
            cv2.waitKey(0)
        except Exception as e:
            pass