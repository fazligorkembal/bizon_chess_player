import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import random

# ==========================
# GLOBAL PARAMS (auto tuned)
# ==========================
blur_ksize = 3
blur_sigmaX = 0
blur_sigmaY = 0
adaptive_thresh_block_size = 31
adaptive_thresh_C = 5


# ==========================
# UTILS
# ==========================
def order_points(pts):
    pts = pts.reshape(4, 2)
    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect


def auto_canny(image, sigma=0.33):
    v = np.median(image)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    return cv2.Canny(image, lower, upper)


def score_detection(approx, contour, img_area):
    if approx is None or len(approx) != 4:
        return 0

    area = cv2.contourArea(contour)
    area_ratio = area / img_area

    if not (0.2 < area_ratio < 0.9):
        return 0

    peri = cv2.arcLength(contour, True)
    approx_peri = cv2.arcLength(approx, True)

    return approx_peri / peri


# ==========================
# MAIN PIPELINE
# ==========================
def get_cropped_image(image):
    global adaptive_thresh_block_size, adaptive_thresh_C

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # === CLAHE (lux normalize)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)

    h, w = gray.shape
    img_area = h * w

    adaptive_thresh_block_size = max(15, (min(h, w) // 20) | 1)

    blur = cv2.GaussianBlur(
        gray,
        (blur_ksize, blur_ksize),
        blur_sigmaX,
        sigmaY=blur_sigmaY
    )

    best_score = 0
    best_approx = None

    for C in range(2, 12):
        thresh = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            adaptive_thresh_block_size,
            C
        )

        edges = auto_canny(thresh)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        edges = cv2.dilate(edges, kernel, iterations=2)
        edges = cv2.erode(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for c in contours:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            score = score_detection(approx, c, img_area)

            if score > best_score:
                best_score = score
                best_approx = approx

    if best_approx is None or len(best_approx) != 4:
        return None

    # === WARP PERSPECTIVE ===
    src_pts = order_points(best_approx)

    # hedef boyut (kare yapıyoruz)
    widthA = np.linalg.norm(src_pts[2] - src_pts[3])
    widthB = np.linalg.norm(src_pts[1] - src_pts[0])
    maxWidth = int(max(widthA, widthB))

    heightA = np.linalg.norm(src_pts[1] - src_pts[2])
    heightB = np.linalg.norm(src_pts[0] - src_pts[3])
    maxHeight = int(max(heightA, heightB))

    dst_pts = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]
    ], dtype="float32")

    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

    return warped


def get_square_bboxes(board_img, border_ratio=0.08, inner_crop_ratio=0.03):
    """
    return:
        squares_bboxes : list of (x1, y1, x2, y2) 64 adet
    """

    h, w = board_img.shape[:2]

    bx = int(w * border_ratio)
    by = int(h * border_ratio)

    inner_w = w - 2 * bx
    inner_h = h - 2 * by

    square_w = inner_w / 8.0
    square_h = inner_h / 8.0

    boxes = []

    for row in range(8):
        for col in range(8):
            x1 = bx + col * square_w
            y1 = by + row * square_h
            x2 = bx + (col + 1) * square_w
            y2 = by + (row + 1) * square_h

            # === inner crop (frame / yazı / border kaçsın)
            dx = (x2 - x1) * inner_crop_ratio
            dy = (y2 - y1) * inner_crop_ratio

            x1 += dx
            y1 += dy
            x2 -= dx
            y2 -= dy

            boxes.append((
                int(x1), int(y1),
                int(x2), int(y2)
            ))

    return boxes

def extract_square_images(
    board_img,
    out_size=64,
    apply_clahe=True
):
    squares = []
    boxes = get_square_bboxes(board_img)

    for (x1, y1, x2, y2) in boxes:
        sq = board_img[y1:y2, x1:x2]

        sq = cv2.resize(sq, (out_size, out_size))

        if apply_clahe:
            gray = cv2.cvtColor(sq, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(2.0, (8, 8))
            gray = clahe.apply(gray)
            sq = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        squares.append(sq)

    return squares

def debug_draw_square_boxes(board_img):
    debug = board_img.copy()
    boxes = get_square_bboxes(board_img)

    for i, (x1, y1, x2, y2) in enumerate(boxes):
        color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cv2.putText(debug, str(i), (cx - 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    cv2.imshow("Square BBoxes", debug)
    cv2.waitKey(1)


# ==========================
# ROS NODE
# ==========================
class RGBCameraCalibration(Node):
    def __init__(self):
        super().__init__('rgb_camera_calibration')
        self.get_logger().info('RGB Camera Calibration Node started')
        self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)

            if msg.encoding in ['rgb8', 'bgr8']:
                channels = 3
            elif msg.encoding in ['rgba8', 'bgra8']:
                channels = 4
            else:
                channels = 3

            frame = np_arr.reshape((msg.height, msg.width, channels))

            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            board = get_cropped_image(frame)
            if board is None:
                return

            #extract_square_images(board)
            debug_draw_square_boxes(board)

        except Exception as e:
            self.get_logger().error(str(e))


# ==========================
# MAIN
# ==========================
def main(args=None):
    rclpy.init(args=args)
    node = RGBCameraCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
