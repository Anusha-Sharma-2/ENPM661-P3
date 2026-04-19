import numpy as np
import cv2
import math

WIDTH = 400
HEIGHT = 200

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (150, 150, 150)
RED = (0, 0, 255)
BLUE = (255, 0, 0)

def generate_map(robot_radius, clearance):
    map_img = np.full((HEIGHT, WIDTH, 3), 255, dtype=np.uint8)
    bloat = int(robot_radius + clearance)
    obs_mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    
    def pt(x, y):
        return (int(x), int(HEIGHT - 1 - y))

    # squares
    # Centers: (45, 42), (133.5, 155), (220, 174)
    for cx, cy in [(45, 42), (133.5, 155), (220, 174)]:
        cv2.rectangle(obs_mask, pt(cx-7.5, cy+7.5), pt(cx+7.5, cy-7.5), 255, -1)

    # right slide vertical wall - after 3rd square
    cv2.rectangle(obs_mask, pt(295, 200), pt(300, 55), 255, -1)
    
    # first llanting line - at box 1 center x
    # x=45 y=200 140 -30
    x1_start, y1_start = 45, 200
    x1_end = x1_start + 140 * math.cos(math.radians(-60))
    y1_end = y1_start + 140 * math.sin(math.radians(-60))
    cv2.line(obs_mask, pt(x1_start, y1_start), pt(x1_end, y1_end), 255, 5)
    
    # second slanting line - at box 2 center x
    # x=133.5 y=0 135 120
    x2_start, y2_start = 133.5, 0
    x2_end = x2_start + 135 * math.cos(math.radians(60))
    y2_end = y2_start + 135 * math.sin(math.radians(60))
    cv2.line(obs_mask, pt(x2_start, y2_start), pt(x2_end, y2_end), 255, 5)

    # bloat
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (bloat*2+1, bloat*2+1))
    bloated_mask = cv2.dilate(obs_mask, kernel)

    map_img[bloated_mask == 255] = GREY
    map_img[obs_mask == 255] = BLACK
    cv2.rectangle(map_img, (0, 0), (WIDTH-1, HEIGHT-1), GREY, bloat*2)
    cv2.circle(map_img, pt(0, 100), bloat + 2, WHITE, -1)
    
    return map_img

def is_valid_node(x, y, map_img):
    if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT: return False
    row = int(HEIGHT - 1 - y)
    col = int(x)
    if row >= HEIGHT or col >= WIDTH or row < 0 or col < 0: return False
    b, g, r = map_img[row, col]
    return not ((b, g, r) == BLACK or (b, g, r) == GREY)