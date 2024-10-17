import cv2


def main():
    vid = cv2.VideoCapture(0)

    while 1:
        ret, frame = vid.read()
        if ret is False:
            break

        cv2.imshow("Detected Markers", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()
