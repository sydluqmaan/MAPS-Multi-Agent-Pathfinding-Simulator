import pygame
import sys
import os

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (204, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)
MAGENTA = (255, 0, 255)
CYAN = (0, 255, 255)
BRIGHT_BLACK = (128, 128, 128)
BRIGHT_RED = (255, 69, 0)
BRIGHT_GREEN = (50, 205, 50)
BRIGHT_YELLOW = (255, 255, 224)
BRIGHT_BLUE = (0, 191, 255)
BRIGHT_MAGENTA = (255, 20, 147)
BRIGHT_CYAN = (0, 255, 255)
BRIGHT_WHITE = (255, 250, 250)
DARK_GRAY = (169, 169, 169)
DARK_RED = (139, 0, 0)
DARK_GREEN = (0, 100, 0)
DARK_BLUE = (0, 0, 139)
WHITE_A = (255,255,255, 75)
YELLOW_A = (255, 255, 0, 30)
RED_A = (204, 0, 75)

color_list = [
    (204, 0, 0),       # RED
    (0, 0, 255),       # BLUE
    (255, 0, 255),     # MAGENTA
    (128, 128, 128),   # BRIGHT_BLACK
    (255, 69, 0),      # BRIGHT_RED
    (50, 205, 50),     # BRIGHT_GREEN
    (255, 20, 147),    # BRIGHT_MAGENTA
    (169, 169, 169),   # DARK_GRAY
    (139, 0, 0),       # DARK_RED
    (0, 100, 0),       # DARK_GREEN
    (0, 0, 139),       # DARK_BLUE
    (34, 139, 34),     # FOREST_GREEN
    (47, 79, 79),      # DARK_SLATE_GRAY
    (128, 0, 128),     # PURPLE
    (0, 128, 128),     # TEAL
    (105, 105, 105),   # DIM_GRAY
    (72, 61, 139),     # DARK_SLATE_BLUE
    (139, 69, 19),     # SADDLE_BROWN
    (85, 107, 47),     # DARK_OLIVE_GREEN
    (25, 25, 112)      # MIDNIGHT_BLUE
]

if getattr(sys, 'frozen', False):
    current_dir = os.path.dirname(sys.executable)
else:
    current_dir = os.path.dirname(__file__)

font1 = os.path.join(current_dir, 'assets', 'ShareTechMono-Regular.ttf')