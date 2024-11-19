import time
import cv2

import os

os.environ["QT_QPA_PLATFORM"] = "xcb"

now = 0
prev = 0
interval = 1


def main():
    global prev, now
    cap = cv2.VideoCapture(0)

    i = 0
    while 1:
        ret, frame = cap.read()
        now = time.time()
        cv2.imshow("win", frame)
        cv2.waitKey(1)
        if now - prev >= interval:
            prev = now
            if ret:
                cv2.imwrite(f"calibration/cap_{i}.jpg", frame)
                print(f"captured_{i}")
                i += 1


if __name__ == "__main__":
    main()
