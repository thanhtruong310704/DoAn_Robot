import tkinter as tk
from giaodien import RobotApp

if __name__ == "__main__":
    root = tk.Tk()
    
    # Khởi tạo Giao diện từ file
    app = RobotApp(root)
    
    # Gán sự kiện tắt cửa sổ
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    
    # Chạy vòng lặp
    root.mainloop() 