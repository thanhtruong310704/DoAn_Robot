import tkinter as tk
from tkinter import scrolledtext, messagebox
from PIL import Image, ImageTk
import cv2
import numpy as np
import threading
import time

import config as cfg
import kinematic as kin
import control as ctrl

class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Đồ Án Robot - Nhóm 5")
        
        
        self.win_width = 1000  
        self.win_height = 700 
        self.root.geometry(f"{self.win_width}x{self.win_height}")

        self.mode_var = tk.StringVar(value="Mode 2")
        self.trigger_calib = False
        self.video_running = True
        
        # --- BANNER ---
        self.banner_frame = tk.Frame(root, height=100, bg="#f0f0f0")
        self.banner_frame.pack(side=tk.TOP, fill=tk.X)
        self.banner_frame.pack_propagate(False)
        
        try:
            img_path = "XLA/logo.png" 
            pil_img = Image.open(img_path)
            base_width = self.win_width
            w_percent = (base_width / float(pil_img.size[0]))
            h_size = int((float(pil_img.size[1]) * float(w_percent)))
            if h_size > 100: h_size = 100
            pil_img = pil_img.resize((self.win_width, h_size), Image.Resampling.LANCZOS)
            self.tk_banner = ImageTk.PhotoImage(pil_img)
            lbl_img = tk.Label(self.banner_frame, image=self.tk_banner, bg="#f0f0f0")
            lbl_img.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        except Exception as e: 
            print("Banner load error:", e)
            tk.Label(self.banner_frame, text="ROBOT CONTROL PANEL", font=("Arial", 20, "bold")).pack(expand=True)

        conn_frame = tk.LabelFrame(root, text="Serial", padx=10, pady=5)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(conn_frame, text="COM:").pack(side=tk.LEFT)
        self.ent_port = tk.Entry(conn_frame, width=8)
        self.ent_port.insert(0, cfg.DEFAULT_PORT)
        self.ent_port.pack(side=tk.LEFT, padx=5)

        tk.Label(conn_frame, text="Baud:").pack(side=tk.LEFT)
        self.ent_baud = tk.Entry(conn_frame, width=8)
        self.ent_baud.insert(0, cfg.DEFAULT_BAUD)
        self.ent_baud.pack(side=tk.LEFT, padx=5)

        self.btn_connect = tk.Button(conn_frame, text="CONNECT", bg="#4CAF50", fg="white", command=self.do_connect)
        self.btn_connect.pack(side=tk.LEFT, padx=10)

        tk.Label(conn_frame, text="|  CHẾ ĐỘ:").pack(side=tk.LEFT, padx=10)
        tk.Radiobutton(conn_frame, text="1. AUTO", variable=self.mode_var, value="Mode 1", command=self.update_ui_mode).pack(side=tk.LEFT)
        tk.Radiobutton(conn_frame, text="2. DEBUG", variable=self.mode_var, value="Mode 2", command=self.update_ui_mode).pack(side=tk.LEFT)
        tk.Radiobutton(conn_frame, text="3. LIST", variable=self.mode_var, value="Mode 3", command=self.update_ui_mode).pack(side=tk.LEFT)

        mid_frame = tk.Frame(root)
        mid_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.cam_frame = tk.LabelFrame(mid_frame, text="Camera", width=500)
        self.cam_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.lbl_cam = tk.Label(self.cam_frame, text="[Waiting for Camera...]", bg="black", fg="white")
        self.lbl_cam.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.list_input_frame = tk.LabelFrame(mid_frame, text="NHẬP TỌA ĐỘ", width=400, bg="#FFF3E0")
        
        hdr_frm = tk.Frame(self.list_input_frame, bg="#FFF3E0")
        hdr_frm.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(hdr_frm, text="MÀU", width=8, bg="#FFF3E0", font="bold").pack(side=tk.LEFT)
        tk.Label(hdr_frm, text="X", width=8, bg="#FFF3E0", font="bold").pack(side=tk.LEFT, padx=2)
        tk.Label(hdr_frm, text="Y", width=8, bg="#FFF3E0", font="bold").pack(side=tk.LEFT, padx=2)
        tk.Label(hdr_frm, text="Z", width=8, bg="#FFF3E0", font="bold").pack(side=tk.LEFT, padx=2)

        self.manual_coords = {}
        colors = ["red", "yellow", "green", "blue", "purple"]
        default_vals = [("0","200","15"), ("200","-50","15"), ("0","210","15"), ("-50","210","15"), ("-100","210","15")] 

        for i, col in enumerate(colors):
            row_f = tk.Frame(self.list_input_frame, bg="#FFF3E0")
            row_f.pack(fill=tk.X, pady=2, padx=5)
            lbl = tk.Label(row_f, text=f"{col.upper()}", width=8, anchor='w', bg="#FFF3E0", fg=col if col!="yellow" else "#FBC02D", font="bold")
            lbl.pack(side=tk.LEFT)
            ent_x = tk.Entry(row_f, width=8); ent_x.insert(0, default_vals[i][0]); ent_x.pack(side=tk.LEFT, padx=2)
            ent_y = tk.Entry(row_f, width=8); ent_y.insert(0, default_vals[i][1]); ent_y.pack(side=tk.LEFT, padx=2)
            ent_z = tk.Entry(row_f, width=8); ent_z.insert(0, default_vals[i][2]); ent_z.pack(side=tk.LEFT, padx=2)
            self.manual_coords[col] = (ent_x, ent_y, ent_z)

        tk.Button(self.list_input_frame, text="CHẠY LIST ĐÃ NHẬP", bg="#FF5722", fg="white", font=("Arial", 11, "bold"), 
                  command=self.start_manual_list_job).pack(fill=tk.X, padx=10, pady=20)

        right_panel = tk.Frame(mid_frame, width=300)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))

        log_frame = tk.LabelFrame(right_panel, text="LOG ĐIỀU KHIỂN ", width=250)
        log_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.txt_log = scrolledtext.ScrolledText(log_frame, width=30, height=10, state='disabled')
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        cfg.gui_log_widget = self.txt_log

        calib_frame = tk.LabelFrame(right_panel, text="OFFSET", bg="#E0F7FA")
        calib_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=10)

        # Hàng 1: Scale X, Shift X
        r1 = tk.Frame(calib_frame, bg="#E0F7FA"); r1.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(r1, text="Scale X:", bg="#E0F7FA").pack(side=tk.LEFT)
        self.ent_scale_x = tk.Entry(r1, width=6); self.ent_scale_x.pack(side=tk.LEFT, padx=2)
        self.ent_scale_x.insert(0, str(cfg.ROBOT_SCALE_X)) # Load từ config
        
        tk.Label(r1, text="Shift X:", bg="#E0F7FA").pack(side=tk.LEFT, padx=(10,0))
        self.ent_shift_x = tk.Entry(r1, width=6); self.ent_shift_x.pack(side=tk.LEFT, padx=2)
        self.ent_shift_x.insert(0, str(cfg.ROBOT_SHIFT_X))

        # Hàng 2: Scale Y, Shift Y
        r2 = tk.Frame(calib_frame, bg="#E0F7FA"); r2.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(r2, text="Scale Y:", bg="#E0F7FA").pack(side=tk.LEFT)
        self.ent_scale_y = tk.Entry(r2, width=6); self.ent_scale_y.pack(side=tk.LEFT, padx=2)
        self.ent_scale_y.insert(0, str(cfg.ROBOT_SCALE_Y))
        
        tk.Label(r2, text="Shift Y:", bg="#E0F7FA").pack(side=tk.LEFT, padx=(10,0))
        self.ent_shift_y = tk.Entry(r2, width=6); self.ent_shift_y.pack(side=tk.LEFT, padx=2)
        self.ent_shift_y.insert(0, str(cfg.ROBOT_SHIFT_Y))

        # Nút Update
        tk.Button(calib_frame, text="DPD OFFSET", bg="#009688", fg="white", font=("Arial", 9, "bold"),
                  command=self.update_robot_offset).pack(fill=tk.X, padx=10, pady=5)

        bot_frame = tk.Frame(root, pady=5)
        bot_frame.pack(fill=tk.X, padx=10)

        self.frm_manual = tk.LabelFrame(bot_frame, text="DEBUG / MANUAL CONTROL", padx=5, pady=5)
        self.frm_manual.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        btn_opts = {'font': ('Arial', 9, 'bold'), 'width': 6, 'height': 2}
        
        tk.Button(self.frm_manual, text="HOME", bg="#8BC34A", command=lambda: self.run_thread(ctrl.move_robot_wait, cfg.HOME_POS, 4.0), **btn_opts).pack(side=tk.LEFT, padx=2)
        tk.Button(self.frm_manual, text="BOX",  bg="#03A9F4", command=lambda: self.run_thread(ctrl.move_robot_wait, cfg.BOX_POS, 4.0), **btn_opts).pack(side=tk.LEFT, padx=2)
        tk.Button(self.frm_manual, text="PICK", command=lambda: ctrl.control_pump(True), width=6, bg="#FFCCBC").pack(side=tk.LEFT, padx=2)
        tk.Button(self.frm_manual, text="PLACE", command=lambda: ctrl.control_pump(False), width=6, bg="#C8E6C9").pack(side=tk.LEFT, padx=2)
        
        tk.Frame(self.frm_manual, width=10).pack(side=tk.LEFT) 
        tk.Button(self.frm_manual, text="GA", command=self.do_ga_btn, width=5, bg="#E1BEE7").pack(side=tk.LEFT, padx=2)
        tk.Button(self.frm_manual, text="GP", command=self.do_gp_btn, width=5, bg="#E1BEE7").pack(side=tk.LEFT, padx=2)
        tk.Button(self.frm_manual, text="AS5600", command=self.do_as5600_btn, width=6, bg="#D1C4E9").pack(side=tk.LEFT, padx=2)
        tk.Label(self.frm_manual, text="| X:").pack(side=tk.LEFT)
        self.ent_x = tk.Entry(self.frm_manual, width=4); self.ent_x.pack(side=tk.LEFT)
        tk.Label(self.frm_manual, text="Y:").pack(side=tk.LEFT)
        self.ent_y = tk.Entry(self.frm_manual, width=4); self.ent_y.pack(side=tk.LEFT)
        tk.Label(self.frm_manual, text="Z:").pack(side=tk.LEFT)
        self.ent_z = tk.Entry(self.frm_manual, width=4); self.ent_z.pack(side=tk.LEFT)
        tk.Button(self.frm_manual, text="RUN", bg="#FFC107", command=self.do_move_xyz, width=4).pack(side=tk.LEFT, padx=5)

        self.frm_auto = tk.LabelFrame(bot_frame, text="AUTO CAMERA", padx=5, pady=5)
        self.frm_auto.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10,0))
        
        tk.Button(self.frm_auto, text="CALIB", bg="#FF9800", fg="white", command=self.do_calib_btn, **btn_opts).pack(side=tk.LEFT, padx=5)
        self.btn_run_auto = tk.Button(self.frm_auto, text="BẮT ĐẦU\nGẮP", bg="#F44336", fg="white", command=self.start_auto_btn, width=12, height=2, font=('Arial', 9, 'bold'))
        self.btn_run_auto.pack(side=tk.LEFT, padx=5)
        
        # Start Vision Thread
        threading.Thread(target=self.vision_loop_thread, daemon=True).start()
        self.update_ui_mode()

    # --- HÀM XỬ LÝ SỰ KIỆN ---
    def update_robot_offset(self):
        try:
            # Đọc giá trị từ ô nhập
            s_x = float(self.ent_scale_x.get())
            sh_x = float(self.ent_shift_x.get())
            s_y = float(self.ent_scale_y.get())
            sh_y = float(self.ent_shift_y.get())

            # Cập nhật vào Biến toàn cục
            cfg.ROBOT_SCALE_X = s_x
            cfg.ROBOT_SHIFT_X = sh_x
            cfg.ROBOT_SCALE_Y = s_y
            cfg.ROBOT_SHIFT_Y = sh_y
            
            ctrl.log_print("ĐÃ CẬP NHẬT OFFSET")
            ctrl.log_print(f"X: x*{s_x:.3f} + {sh_x:.1f}")
            ctrl.log_print(f"Y: y*{s_y:.3f} + {sh_y:.1f}")
            messagebox.showinfo("Thành công", "Đã cập nhật thông số Calib mới")
            
        except ValueError:
            messagebox.showerror("Lỗi", "Vui lòng chỉ nhập số thực")

    def update_ui_mode(self):
        mode = self.mode_var.get()
        if mode == "Mode 3": 
            self.cam_frame.pack_forget() 
            self.list_input_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True) 
            self.frm_auto.config(text="CHẾ ĐỘ LIST")
            self.btn_run_auto.config(state="disabled", bg="gray")
        else:
            self.list_input_frame.pack_forget() 
            self.cam_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True) 
            self.frm_auto.config(text="AUTO CAMERA")
            self.btn_run_auto.config(state="normal", bg="#F44336")

    def do_ga_btn(self):
        angles = ctrl.read_current_angles()
        if angles: ctrl.log_print(f"Góc: q1={angles[0]:.2f}, q2={angles[1]:.2f}, q3={angles[2]:.2f}")
        else: ctrl.log_print("[GA]Lỗi")

    def do_gp_btn(self):
        pos = ctrl.get_current_position()
        if pos: ctrl.log_print(f"Tọa độ: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f}")
        else: ctrl.log_print("[GP]Lỗi")
    def do_as5600_btn(self):
        if cfg.ser and cfg.ser.is_open:
            try:
                cfg.ser.write(b'A')
                ctrl.log_print("Đã gửi 'A' (AS5600)")
            except Exception as e:
                ctrl.log_print(f"Lỗi gửi Serial: {e}")
        else:
            ctrl.log_print("Chưa kết nối Serial!")
            messagebox.showwarning("Lỗi", "Vui lòng kết nối Serial trước")

    def start_manual_list_job(self):
        if cfg.execution_running:
            ctrl.log_print("Robot busy")
            return
        temp_list = []
        try:
            for name, (ex, ey, ez) in self.manual_coords.items():
                # Lấy giá trị nhập 
                x_val = float(ex.get())
                y_val = float(ey.get())
                z_val = float(ez.get())
                
                # Bỏ qua nếu là 0,0,0
                if x_val == 0 and y_val == 0 and z_val == 0: continue
                
                #ÁP DỤNG BÙ SAI SỐ ---
                x_cmd, y_cmd, z_cmd = ctrl.apply_robot_correction(x_val, y_val, z_val)
                
                # Lưu tọa độ CMD vào danh sách
                temp_list.append({'name': name, 'coords': (x_cmd, y_cmd, z_cmd)})
                
        except ValueError:
            messagebox.showerror("Lỗi", "Chỉ nhập số thực!")
            return
            
        if not temp_list:
            ctrl.log_print("Danh sách rỗng!"); return
            
        cfg.saved_job_list = temp_list
        ctrl.log_print(f"Bắt đầu chạy List ({len(cfg.saved_job_list)} điểm)...")
        
        # Log ra để kiểm tra
        ctrl.log_print(f"-> Mode: Offset ON")
        
        cfg.execution_running = True
        threading.Thread(target=ctrl.execution_thread_func).start()

    def do_move_xyz(self):
        try:
            #Lấy tọa độ mong muốn 
            x_in = float(self.ent_x.get())
            y_in = float(self.ent_y.get())
            z_in = float(self.ent_z.get())
            # Hàm này sẽ dùng SCALE/SHIFT bạn vừa nhập ở trên
            x_cmd, y_cmd, z_cmd = ctrl.apply_robot_correction(x_in, y_in, z_in)

            # Kiểm tra IK 
            if kin.IK(x_cmd, y_cmd, z_cmd)[0] is None: 
                messagebox.showerror("Lỗi", f"Ngoài tầm với (Sau khi Offset)!")
            else:
                # In log 
                ctrl.log_print(f"User: ({x_in}, {y_in})")
                ctrl.log_print(f"-> Cmd: ({x_cmd:.1f}, {y_cmd:.1f})")
                
                #Gửi lệnh
                self.run_thread(ctrl.move_robot_wait, (x_cmd, y_cmd, z_cmd), 4.0)
                
        except ValueError: messagebox.showerror("Lỗi", "Nhập số hợp lệ!")

    def do_connect(self):
        port = self.ent_port.get()
        baud = int(self.ent_baud.get())
        try:
            cfg.ser = ctrl.serial.Serial(port, baud, timeout=1)
            time.sleep(2); cfg.ser.reset_input_buffer()
            ctrl.log_print(f"Đã kết nối {port}")
            self.btn_connect.config(text="CONNECTED", bg="gray", state="disabled")
            ctrl.log_print("Về HOME...")
            self.run_thread(ctrl.move_robot_wait, cfg.HOME_POS, 2.0)
        except Exception as e:
            ctrl.log_print(f"Lỗi kết nối: {e}")
            messagebox.showwarning("Lỗi", "Không tìm thấy COM")

    def run_thread(self, func, *args):
        threading.Thread(target=func, args=args).start()

    def do_calib_btn(self):
        if self.mode_var.get() == "Mode 1":
            self.trigger_calib = True
            ctrl.log_print("Đặt bàn cờ vào camera")
        else: messagebox.showinfo("Lỗi", "Chọn Mode 1 để Calib")

    def start_auto_btn(self):
        if self.mode_var.get() != "Mode 1":
            messagebox.showinfo("Lỗi", "Chọn Mode 1 để chạy Cam")
            return
        if not cfg.is_calibrated:
            messagebox.showwarning("Lỗi ", "Chưa Calib Camera!")
            return
        if cfg.execution_running:
            ctrl.log_print("Đang chạy"); return

        cfg.saved_job_list = []
        with cfg.coord_lock:
            for n, c in cfg.live_coordinates.items():
                if c: cfg.saved_job_list.append({'name': n, 'coords': c})
        
        if cfg.saved_job_list:
            ctrl.log_print(f"Cam thấy {len(cfg.saved_job_list)} vật. Bắt đầu gắp...")
            cfg.execution_running = True
            threading.Thread(target=ctrl.execution_thread_func).start()
        else: ctrl.log_print("Không thấy vật nào!")

    def vision_loop_thread(self):
        REAL_OBJ_HEIGHT = 5.0  # Chiều cao vật (mm)
    
        MAX_BUFFER_SIZE = 10     # Lấy trung bình 10 ảnh gần nhất
        JUMP_THRESHOLD = 20.0   
        coord_buffers = {name: [] for name in cfg.color_ranges} # Tạo buffer lưu trữ

        cap = cv2.VideoCapture(cfg.CAMERA_INDEX)
        
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, 250)       # Chỉnh nét 
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.5) 
        cap.set(cv2.CAP_PROP_EXPOSURE, -4)     # Giảm sáng thủ công 

        K_inv = np.linalg.inv(cfg.mtx)

        while self.video_running:
            ret, frame = cap.read()
            if not ret: 
                time.sleep(0.05)
                continue
            
            frame = cv2.undistort(frame, cfg.mtx, cfg.dist, None, cfg.mtx)

            if self.trigger_calib:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ret_b, corners = cv2.findChessboardCorners(gray, cfg.chessboard_size, None)
                if ret_b:
                    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), 
                                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    _, rvec, tvec = cv2.solvePnP(cfg.objp, corners2, cfg.mtx, cfg.dist)
                    R_cam, _ = cv2.Rodrigues(rvec)
                    with cfg.coord_lock:
                        cfg.R_inv_saved = R_cam.T
                        cfg.t_world_saved = -R_cam.T @ tvec.flatten()
                        cfg.is_calibrated = True
                        print(f"ĐỘ CAO CAMERA ĐO ĐƯỢC: {cfg.t_world_saved[2]:.2f} mm")
                    ctrl.log_print("CALIB THANH CONG")
                else: 
                    ctrl.log_print("Khong tim thay ban co")
                self.trigger_calib = False

            if cfg.is_calibrated:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                current_frame_coords = {} 

                for name, ((lower, upper), box_color) in cfg.color_ranges.items():
                    # Tạo mặt nạ màu
                    mask = cv2.inRange(hsv, lower, upper)
                    # Lọc nhiễu
                    mask = cv2.erode(mask, None, iterations=1)
                    mask = cv2.dilate(mask, None, iterations=3)
                    
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if contours:
                        c = max(contours, key=cv2.contourArea)
                        
                        # Chỉ bắt vật đủ lớn
                        if cv2.contourArea(c) > 800:
                            
                            # Hàmrả về: ((center_x, center_y), (width, height), angle)
                            rect = cv2.minAreaRect(c)
                            (center_x, center_y), (w_rect, h_rect), angle = rect
                            
                            cx, cy = int(center_x), int(center_y)

                            # Lấy 4 đỉnh hộp 
                            box = cv2.boxPoints(rect)
                            box = np.int32(box)
                            
                            # Vẽ hộp xoay bao quanh vật
                            cv2.drawContours(frame, [box], 0, box_color, 2)
                            # Vẽ tâm vật
                            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

                            #Pixel -> Camera 
                            pt = K_inv @ np.array([cx, cy, 1.0])
                            dir_cam = np.array([pt[0], pt[1], 1.0])
                            
                            #Camera -> World 
                            dir_world = cfg.R_inv_saved @ dir_cam
                            
                            if abs(dir_world[2]) > 1e-6:
                                scale = (REAL_OBJ_HEIGHT - cfg.t_world_saved[2]) / dir_world[2]
                                X, Y = (cfg.t_world_saved + scale * dir_world)[:2]
                                
                                #World -> Robot 
                                P_new = cfg.R @ np.array([X, Y, 0.0]) + cfg.p
                                xr, yr, zr, _ = (cfg.Tr_inv @ np.array([[P_new[0]], [P_new[1]], [P_new[2]], [1]])).flatten()
                                
                                # Đảo trục Y 
                                yr = -yr 
                                SCALE_X = 1.054  # Nếu robot đi ngắn hơn thực tế -> Tăng số này (>1)
                                SHIFT_X =-10.834  # Dời gốc tọa độ X

                                SCALE_Y = 1.062   # Nếu robot đi ngắn hơn thực tế -> Tăng số này
                                SHIFT_Y = -14.092 # Dời gốc tọa độ Y

                                xr = (xr * SCALE_X) + SHIFT_X
                                yr = (yr * SCALE_Y) + SHIFT_Y
                                
                                buffer = coord_buffers[name]
                                current_point = np.array([xr, yr])
                                
                                if len(buffer) > 0:
                                    prev_avg = np.mean(buffer, axis=0)
                                    if np.linalg.norm(current_point - prev_avg) > JUMP_THRESHOLD:
                                        buffer.clear()
                                    buffer.append(current_point)
                                else:
                                    buffer.append(current_point)

                                if len(buffer) > MAX_BUFFER_SIZE: 
                                    buffer.pop(0)

                                avg_x = np.mean([b[0] for b in buffer])
                                avg_y = np.mean([b[1] for b in buffer])
                                
                                std_dev = np.std(buffer, axis=0)
                                stability = np.max(std_dev) # Độ lệch lớn nhất

                                status = "[OK]" if stability < 2.0 else "[Wait]"
                                info_text = f"{name}: {avg_x:.0f}, {avg_y:.0f}"
                                cv2.putText(frame, info_text, (cx - 60, cy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                                cv2.putText(frame, f"A:{int(angle)} {status}", (cx - 40, cy + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1)
                                
                                x_cmd, y_cmd, z_cmd = ctrl.apply_robot_correction(avg_x, avg_y, zr)
                                
                                current_frame_coords[name] = (x_cmd, y_cmd, z_cmd)

                    else:
                        # Nếu mất dấu vật -> Xóa buffer 
                        coord_buffers[name] = []

                # Cập nhật biến toàn cục
                with cfg.coord_lock:
                    for name in cfg.color_ranges: 
                        cfg.live_coordinates[name] = current_frame_coords.get(name, None)
            else:
                
                cv2.putText(frame, "CHUA CALIB", (30, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Cập nhật ảnh lên giao diện Tkinter
            self.update_gui_image(frame)
        
        cap.release()
    
    def update_gui_image(self, frame):
        h, w = frame.shape[:2]
        new_w = 500; new_h = int(h * (new_w / w))
        frame = cv2.resize(frame, (new_w, new_h))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img)
        self.root.after(0, lambda: self._set_image(imgtk))

    def _set_image(self, imgtk):
        self.lbl_cam.config(image=imgtk)
        self.lbl_cam.image = imgtk

    def on_close(self):
        if messagebox.askokcancel("Thông báo", "Thoát chương trình?"):
            cfg.stop_program = True
            self.video_running = False
            if cfg.ser and cfg.ser.is_open: cfg.ser.close()
            self.root.after(100, self.root.destroy)