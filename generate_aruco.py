import cv2


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for i in range(50):
        marker = cv2.aruco.generateImageMarker(dictionary, i, 200)
        cv2.imwrite(f"marker/CV2_ARUCO_MARKER_{i}.png", marker)


if __name__ == "__main__":
    main()
