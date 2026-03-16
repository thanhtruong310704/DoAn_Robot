import cv2
import numpy as np
import glob

# --- Cấu hình ---
images = glob.glob('XLA/img_hieuchinh/*.jpg') 
chessboard_size = (14, 11)     # Số giao điểm (ngang, dọc)
square_size = 14.0            # Kích thước ô vuông (mm)

objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # Lưu điểm 3D thực tế
imgpoints = []  # Lưu điểm 2D trên ảnh

first_valid_img = None
first_valid_corners = None

#Chuyển ảnh RGB -> xám & tìm góc bàn cờ 
print(f"Đã tìm thấy {len(images)} ảnh. Bắt đầu xử lý...")

for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    if img is None:
        continue
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Tìm góc bàn cờ
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        
        # Tinh chỉnh tọa độ góc cho chính xác hơn
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)
        
        # Vẽ giao điểm lên ảnh 
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('a', cv2.resize(img, (600, 450)))
        cv2.waitKey(100) # Dừng 100ms mỗi ảnh

        if first_valid_img is None:
            first_valid_img = img
            first_valid_corners = corners2

cv2.destroyAllWindows() 

print("\n KẾT QUẢ HIỆU CHỈNH")
if len(objpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\nMa trận nội tại (Camera Matrix K):")
    print(mtx)
    print("\nHệ số méo (Distortion Coefficients):")
    print(dist.ravel())

    #Lưu thông số camera ---
    np.savez('vison_armrobot/camera_data.npz', mtx=mtx, dist=dist)
    print("\nĐã lưu file")

    if first_valid_img is not None:
        print(" ")
        cv2.imshow('Ket qua:', cv2.resize(first_valid_img, (600, 450))) 
        cv2.waitKey(0)    
    cv2.destroyAllWindows()
else:
    print("Lỗi")
