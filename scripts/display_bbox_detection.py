import sys
import cv2
import numpy as np
import random
import time

# ==========================

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
C_HOLD = None

def get_cropped_image(image):
    global adaptive_thresh_block_size, adaptive_thresh_C, C_HOLD

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

    if C_HOLD is None:
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
                    if len(best_approx) == 4:
                        print(f"Found good approximation with C={C}, score={best_score:.4f}")
                        C_HOLD = C
                        break
    else:
        thresh = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            adaptive_thresh_block_size,
            C_HOLD
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
        return None, None

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

    return warped, M


def get_square_bboxes(board_img, border_ratio=0.08, inner_crop_ratio=0.01):
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

def debug_draw_square_boxes(image, board_img, M=None, isaac_bboxes=None):
    
    boxes = get_square_bboxes(board_img)

    for i, (x1, y1, x2, y2) in enumerate(boxes):
        if M is not None:
            pts = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype="float32").reshape(-1, 1, 2)
            pts = cv2.perspectiveTransform(pts, np.linalg.inv(M))
            pts = pts.astype(int)
            cv2.polylines(image, [pts], isClosed=True, color=(0,255,0), thickness=2)
            center_x = (pts[0][0][0] + pts[2][0][0]) // 2
            center_y = (pts[0][0][1] + pts[2][0][1]) // 2
            cv2.circle(image, (center_x, center_y), 5, (255,0,0), -1)

            for isaac_bbox in isaac_bboxes:
                class_id = int(isaac_bbox["semanticId"])
                x_min = int(isaac_bbox["x_min"])
                y_min = int(isaac_bbox["y_min"])
                x_max = int(isaac_bbox["x_max"])
                y_max = int(isaac_bbox["y_max"])

                isaac_center_x = (x_min + x_max) // 2
                isaac_center_y = (y_min + y_max) // 2
                distance = np.sqrt((center_x - isaac_center_x) ** 2 + (center_y - isaac_center_y) ** 2)

                if distance < 50:
                    cv2.circle(image, (isaac_center_x, isaac_center_y), 5, (0,0,255), -1)
                
        
        
        #color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        #cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
        #cx = (x1 + x2) // 2
        #cy = (y1 + y2) // 2
        #cv2.putText(debug, str(i), (cx - 10, cy),
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    cv2.imshow("Square BBoxes", image)
    cv2.waitKey(0)


if __name__ == "__main__":
    '''
    image_path = f"/home/user/Documents/bizon_chess_player/datasets/test1.jpeg"
    image = cv2.imread(image_path)
    #rotate 90 degrees
    image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    image = cv2.resize(image, (1280, 720))
    
    board_img = get_cropped_image(image)
    if board_img is not None:
        debug_draw_square_boxes(board_img)
    
    '''
    for i in range(2000):
        image_path = f"/home/user/Documents/bizon_chess_player/datasets/chessboard_train/rgb_{i:04d}.png"
        label_path = image_path.replace("rgb_", "bounding_box_2d_tight_").replace(".png", ".npy")
        isaac_bboxes = np.load(label_path, allow_pickle=True)
        image = cv2.imread(image_path)
        image_debug = image.copy()
        start_time = time.time()
        board_img, M = get_cropped_image(image)
        end_time = time.time()
        print(f"Image {i}: Detection took {(end_time - start_time) * 1000:.2f} ms")
        if board_img is not None:
            debug_draw_square_boxes(image, board_img, M, isaac_bboxes)
    