import cv2
import time


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    param = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, param)
    vid = cv2.VideoCapture(0)

    start_time = time.time()
    frame_counter = 0

    while 1:
        frame_counter += 1
        ret, frame = vid.read()
        if ret is False:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        fps = frame_counter / (time.time() - start_time)
        cv2.putText(
            frame,
            f"FPS: {fps:.3f}",
            (30, 30),
            cv2.FONT_HERSHEY_PLAIN,
            1.5,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("Detected Markers", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()
