import time
import tkinter as tk
import config as cfg
import kinematic as kin
import serial
def log_print(*args, **kwargs):
    """Hàm in log thay thế print, vừa in ra console vừa in vào GUI"""
    msg = " ".join(map(str, args))
    print(msg) 
    if cfg.gui_log_widget:
        try:
            cfg.gui_log_widget.config(state='normal')
            cfg.gui_log_widget.insert(tk.END, msg + "\n")
            cfg.gui_log_widget.see(tk.END)
            cfg.gui_log_widget.config(state='disabled')
        except: pass

def send_packet(q1, q2, q3, max_retries=3):
    """Gửi góc xuống Arduino"""
    if not (cfg.ser and getattr(cfg.ser, "is_open", False)):
        log_print(f"q1={q1:.2f}, q2={q2:.2f}, q3={q3:.2f}")
        return True

    msg = f"{q1:.2f},{q2:.2f},{q3:.2f}\n"
    
    for attempt in range(max_retries):
        if cfg.stop_program: return False
        try:
            cfg.ser.reset_input_buffer()
            cfg.ser.write(msg.encode())
            start_wait = time.time()
            while time.time() - start_wait < 1.0:
                if cfg.stop_program: return False
                if cfg.ser.in_waiting:
                    line = cfg.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line == "OK": 
                        time.sleep(0.005) 
                        return True
                time.sleep(0.01)  
            log_print(f"Timeout đã gửi (Lần {attempt+1}/{max_retries})")
        except Exception as e:
            log_print(f" Error Serial: {e}")
            
    log_print("MẤT KẾT NỐI VỚI ROBOT")
    return False

def control_pump(state):
    cmd = "P1\n" if state else "P0\n"
    if cfg.ser and getattr(cfg.ser, "is_open", False):
        try:
            cfg.ser.write(cmd.encode())
            time.sleep(0.1)
        except Exception as e:
            log_print(f"Pump Error: {e}")
    else:
        log_print(f"[SIM PUMP] {'ON' if state else 'OFF'}")

def read_current_angles(timeout=2.0):
    if not (cfg.ser and getattr(cfg.ser, "is_open", False)): return None
    try:
        cfg.ser.reset_input_buffer()
        cfg.ser.write(b"G\n")
        start = time.time()
        while time.time() - start < timeout:
            if cfg.ser.in_waiting > 0:
                line = cfg.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("A:"):
                    return tuple(map(float, line[2:].split(',')))
            time.sleep(0.02)
    except Exception as e:
        log_print(f" Read Angle Error: {e}")
    return None

def get_current_position():
    angles = read_current_angles()
    if angles: return kin.FK(*angles)
    return None

def wait_arrival(timeout=15.0):
    start = time.time()
    if not (cfg.ser and getattr(cfg.ser, "is_open", False)):
        time.sleep(0.1); return True

    log_print("Chờ robot đến vị trí...", end='')
    cfg.ser.reset_input_buffer()
    last_send_time = 0

    while time.time() - start < timeout:
        if cfg.stop_program:
            log_print(" [STOPPED]"); return False
        try:
            if time.time() - last_send_time > 1.5:
                cfg.ser.write(b"C\n")
                last_send_time = time.time()
            if cfg.ser.in_waiting > 0:
                line = cfg.ser.readline().decode('utf-8', errors='ignore').strip()
                if line == "ARRIVED": 
                    log_print("đã đến ")
                    return True
            time.sleep(0.1)  
        except Exception as e:
            log_print(f"\n Wait Error: {e}"); return False
            
    log_print("Timeout!"); return False

def move_robot_wait(target_pos, total_time=2.0):
    dt = 0.1
    traj = kin.cubic_xyz(cfg.current_pos, target_pos, total_time, dt) # tính a0 a1 a2 a3
    
    for p in traj:
        if kin.IK(p[0], p[1], p[2])[0] is None:
            log_print(f"Quỹ đạo qua điểm chết IK: {p}")

    start_time = time.perf_counter()
    for i, p_val in enumerate(traj):
        if cfg.stop_program:
            log_print("\n Di chuyển bị dừng giữa chừng"); return False
        q1, q2, q3 = kin.IK(p_val[0], p_val[1], p_val[2])
        if q1 is None: continue
        if not send_packet(q1, q2, q3):
            log_print("\n Mất kết nối - Dừng di chuyển "); return False
        target_t = start_time + (i + 1) * dt
        while time.perf_counter() < target_t:
            if cfg.stop_program: return False
            time.sleep(0.001)

    if not wait_arrival(): return False
    cfg.current_pos = target_pos
    return True

def execution_thread_func():
    """Logic gắp vật chính"""
    log_print("BẮT ĐẦU GẮP")
    if not cfg.saved_job_list:
        log_print("Danh sách trống!")
        cfg.execution_running = False
        return

    log_print(f"Bắt đầu gắp {len(cfg.saved_job_list)} món...")

    for i, item in enumerate(cfg.saved_job_list):
        if cfg.stop_program: break
            
        name = item['name']
        xr, yr, zr = item['coords']
        target_box = cfg.BOX_POSITIONS.get(name, (cfg.HOME_POS))
        
        test_q1, _, _ = kin.IK(xr, yr, 30.0)
        if test_q1 is None:
            log_print(f"[SKIP] {name} tại ({xr:.1f}, {yr:.1f}) - NGOÀI TẦM VỚI!")
            continue

        log_print(f"\n[{i+1}/{len(cfg.saved_job_list)}] Gắp {name} tại ({xr:.1f}, {yr:.1f})")

        # 1. Đến trên vật
        if not move_robot_wait((xr, yr, 70.0), 5.0): break
        # 2. Xuống gắp
        if not move_robot_wait((xr, yr, 5.0), 5.0): break
        # 3. Bật HUT
        control_pump(True); time.sleep(0.5)
        # 4. Nhấc lên
        if not move_robot_wait((xr, yr, 60.0), 5.0): break
        
        bx, by, bz = target_box
        # 5. Di chuyển đến trên hộp
        if not move_robot_wait((bx,by,50),5.5): break
        # 6. Hạ xuống hộp
        # if not move_robot_wait((bx,by,40),5.5): break
        # 7. Tắt HUT
        control_pump(False); time.sleep(0.5)
        # 8. Nhấc lên lại
        # if not move_robot_wait((bx,by,60),5.5): break
        time.sleep(0.3)

    if not cfg.stop_program:
        log_print("\n Đã gắp xong tất cả")
        move_robot_wait(cfg.HOME_POS, 5.0)
    else:
        log_print("\n ĐÃ DỪNG GIỮA CHỪNG!")
        
    cfg.execution_running = False
    log_print(" Kết thúc tiến trình tự động.")

def apply_robot_correction(x_real, y_real, z_real):
    # Tính toán X, Y theo công thức tuyến tính
    x_cmd = (x_real * cfg.ROBOT_SCALE_X) + cfg.ROBOT_SHIFT_X
    y_cmd = (y_real * cfg.ROBOT_SCALE_Y) + cfg.ROBOT_SHIFT_Y   
    z_cmd = z_real + cfg.ROBOT_OFFSET_Z
    
    return x_cmd, y_cmd, z_cmd