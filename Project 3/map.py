import numpy as np
import cv2

# scaling for project 3
WIDTH = 600
HEIGHT = 250
ROBOT_RADIUS = 5
CLEARANCE = 5
BLOAT = ROBOT_RADIUS + CLEARANCE

# scaling factors
SX = WIDTH / 180
SY = HEIGHT / 50
# ------------------------------

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (150, 150, 150)
RED = (0, 0, 255)
BLUE = (255, 0, 0)

# intersection of halfplanes passed
def rect_hp(x, y, xmin, xmax, ymin, ymax, c):
    # adding sx and sy to scale
    h1 = -x + (xmin * SX - c) <= 0
    h2 =  x - (xmax * SX + c) <= 0
    h3 = -y + (ymin * SY - c) <= 0
    h4 =  y - (ymax * SY + c) <= 0
    
    return h1 and h2 and h3 and h4

# A
def in_A(x, y, c=0):
    left_leg  = rect_hp(x, y, 15, 17, 10, 40, c)
    right_leg = rect_hp(x, y, 28, 30, 10, 40, c)
    top_bar   = rect_hp(x, y, 15, 30, 38, 40, c)
    mid_bar   = rect_hp(x, y, 17, 28, 24, 26, c)
    return left_leg or right_leg or top_bar or mid_bar

# S
def in_S(x, y, c=0):
    top_bar   = rect_hp(x, y, 40, 55, 38, 40, c)
    mid_bar   = rect_hp(x, y, 40, 55, 24, 26, c)
    bot_bar   = rect_hp(x, y, 40, 55, 10, 12, c)
    top_left  = rect_hp(x, y, 40, 42, 26, 38, c)
    bot_right = rect_hp(x, y, 53, 55, 12, 24, c)
    return top_bar or mid_bar or bot_bar or top_left or bot_right

# 9
def in_9(x, y, c=0):
    right_leg = rect_hp(x, y, 78, 80, 10, 40, c)
    top_bar   = rect_hp(x, y, 65, 80, 38, 40, c)
    mid_bar   = rect_hp(x, y, 65, 80, 24, 26, c)
    left_top  = rect_hp(x, y, 65, 67, 26, 38, c)
    return right_leg or top_bar or mid_bar or left_top

# 2
def in_2(x, y, c=0):
    top_bar    = rect_hp(x, y, 90, 105, 38, 40, c)
    upper_right= rect_hp(x, y, 103, 105, 26, 38, c)
    mid_bar    = rect_hp(x, y, 90, 105, 24, 26, c)
    lower_left = rect_hp(x, y, 90, 92, 12, 24, c)
    bot_bar    = rect_hp(x, y, 90, 105, 10, 12, c)
    return top_bar or upper_right or mid_bar or lower_left or bot_bar

# 4
def in_4(x, y, c=0):
    left_leg  = rect_hp(x, y, 115, 117, 24, 40, c)
    mid_bar   = rect_hp(x, y, 115, 130, 24, 26, c)
    right_leg = rect_hp(x, y, 128, 130, 10, 40, c)
    return left_leg or mid_bar or right_leg

# 7
def in_7(x, y, c=0):
    top_bar   = rect_hp(x, y, 140, 155, 38, 40, c)
    right_leg = rect_hp(x, y, 153, 155, 10, 38, c)
    return top_bar or right_leg


def generate_map():
    map_img = np.full((HEIGHT, WIDTH, 3), 255, dtype=np.uint8)
    
    for y in range(HEIGHT):
        for x in range(WIDTH):
            # check map boundaries using bloat
            if x < BLOAT or x >= WIDTH - BLOAT or y < BLOAT or y >= HEIGHT - BLOAT:
                map_img[HEIGHT - 1 - y, x] = GREY
                continue

            # check clearance using blaot
            if in_A(x, y, BLOAT) or in_S(x, y, BLOAT) or in_9(x, y, BLOAT) or \
               in_2(x, y, BLOAT) or in_4(x, y, BLOAT) or in_7(x, y, BLOAT):
                map_img[HEIGHT - 1 - y, x] = GREY
                
            # check obstacle boundaries
            if in_A(x, y, 0) or in_S(x, y, 0) or in_9(x, y, 0) or \
               in_2(x, y, 0) or in_4(x, y, 0) or in_7(x, y, 0):
                map_img[HEIGHT - 1 - y, x] = BLACK

    return map_img

# helper for determining if this is a valid node to move on
def is_valid_node(x, y, map_img):
    # bounds check
    if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
        return False
    
    # cast degrees to integers
    row = int(HEIGHT - 1 - y)
    col = int(x)
    
    row = HEIGHT - 1 - y
    col = x
    b, g, r = map_img[int(row), int(col)]
    # unvalid if pixel is black or grey
    if (b, g, r) == BLACK or (b, g, r) == GREY:
        return False
    return True

# animation work
def animate_search(map_img, explored_nodes, path, filename="animation.mp4"):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(filename, fourcc, 60.0, (WIDTH, HEIGHT))
    
    # node exploration
    for i, node in enumerate(explored_nodes):
        x, y = node
        map_img[HEIGHT - 1 - y, x] = RED
        # only show 100th iteration
        if i % 100 == 0:
            out.write(map_img)

    # final path
    if path:
        for node in path:
            x, y = node
            map_img[HEIGHT - 1 - y, x] = BLUE
            out.write(map_img)

    # wait after path generation
    for _ in range(60):
        out.write(map_img)

    out.release()
    print(f"Animation saved as {filename}")