import cv2


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    param = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, param)
    vid = cv2.VideoCapture(0)

    while 1:
        ret, frame = vid.read()
        if ret is False:
            break

        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imshow("Detected Markers", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()
