import numpy as np
import threading

DEFAULT_PORT = "COM3"
DEFAULT_BAUD = "115200"
CAMERA_INDEX = 1

l1 = 120; l2 = 139.91104; l3 = 140; l4 = 54.04529

HOME_POS = (194, 0, 260)
BOX_POS = (0, 210, 50) 
BOX_POSITIONS = {
    "red":    (0, 230, 80),   
    "yellow": (0, 150, 80),  
    "green":  (-80, 150, 80), 
    "blue":   (-100, 230, 80), 
    "purple": (-80, 150, 80) 
}

#CẤU HÌNH MÀU SẮC (HSV) 
color_ranges = {
    "red":    ((np.array([0, 100, 60]), np.array([10, 255, 255])), (0, 0, 255)),
    "yellow": ((np.array([20, 80, 80]), np.array([35, 255, 255])), (0, 255, 255)),
    "green":  ((np.array([40, 60, 50]), np.array([90, 255, 255])), (0, 255, 0)),
    "blue":   ((np.array([95, 90, 50]), np.array([135, 255, 255])), (255, 0, 0)),
    "purple": ((np.array([135, 80, 60]), np.array([165, 255, 255])), (255, 0, 255)),
}

# CẤU HÌNH CAMERA & BÀN CỜ 
chessboard_size = (14, 11)
square_size = 14.0
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

try:
    data = np.load('XLA/camera_data.npz')
    mtx, dist = data['mtx'], data['dist']
    print("Đã load file camera_data.npz")
except:
    print("Không tìm thấy file camera_data.npz, dùng mặc định.")
    mtx = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=float)
    dist = np.zeros((1, 5))

# Ma trận chuyển đổi Robot - Bàn cờ 
R = np.eye(3)
p = np.array([15.0, 15.0, 0.0])

theta = np.pi/2
Rr = np.array([[np.cos(theta), -np.sin(theta), 0],
               [np.sin(theta),  np.cos(theta), 0],
               [0, 0, 1]])
Pr = np.array([[0], [-50], [0]])
Tr = np.block([[Rr, Pr], [np.zeros((1, 3)), 1]])
Tr_inv = np.linalg.inv(Tr)

ser = None
gui_log_widget = None
stop_program = False
execution_running = False
is_calibrated = False

# Biến lưu trữ tọa độ & Logic
saved_job_list = []
live_coordinates = {name: None for name in color_ranges}
coord_lock = threading.Lock()
current_pos = HOME_POS

# Biến Calib
R_inv_saved = None
t_world_saved = None

ROBOT_SCALE_X = 1.0   
ROBOT_SHIFT_X = 0.0  
ROBOT_SCALE_Y = 1.0   
ROBOT_SHIFT_Y = 0.0
ROBOT_OFFSET_Z=0.0