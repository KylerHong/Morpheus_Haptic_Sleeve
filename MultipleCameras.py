import cv2
import numpy as np
import threading
import time

# Actual camera indices (confirm they are valid and distinct)
camera_indices = [0,1,2,3]  # Swap 0 and 1 to test

# Frame resolution
target_width, target_height = 640, 360

# Shared dictionary to store frames
frame_dict = {}
lock = threading.Lock()

# Camera reading thread
def camera_thread(cam_idx, cap):
    global frame_dict
    while True:
        ret, frame = cap.read()
        if not ret:
            frame = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        else:
            frame = cv2.resize(frame, (target_width, target_height))
        with lock:
            frame_dict[cam_idx] = frame
        time.sleep(0.01)

# Open and configure cameras
video_captures = {}
for cam_idx in camera_indices:
    cap = cv2.VideoCapture(cam_idx, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        print(f"❌ Error: Camera {cam_idx} not accessible.")
        continue
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
    cap.set(cv2.CAP_PROP_FPS, 15)

    video_captures[cam_idx] = cap

# Launch a thread for each camera
for cam_idx, cap in video_captures.items():
    threading.Thread(target=camera_thread, args=(cam_idx, cap), daemon=True).start()

# Create fullscreen display window
cv2.namedWindow("Multi-Cam View", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("Multi-Cam View", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# Main loop to display grid
while True:
    with lock:
        frames = []
        for cam_idx in camera_indices:
            frame = frame_dict.get(cam_idx, np.zeros((target_height, target_width, 3), dtype=np.uint8))
            label = f"Cam {cam_idx}"
            cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            frames.append(frame)

    # Combine into 2x2 grid
    top_row = np.hstack((frames[0], frames[1]))
    bottom_row = np.hstack((frames[2], frames[3]))
    combined = np.vstack((top_row, bottom_row))

    # Show in fullscreen
    cv2.imshow("Multi-Cam View", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
for cap in video_captures.values():
    cap.release()
cv2.destroyAllWindows()
