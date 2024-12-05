import time
import cv2
from aruco_detector import Object, Direction, ObjectType, ArucoDetector

# from broadcaster import Broadcaster
import threading
import asyncio


class Robot:
    detector: ArucoDetector
    # broadcaster: Broadcaster

    def __init__(self):
        pass


def main():
    pass


if __name__ == "__main__":
    robot = Robot()
    markers = [
        Object(
            0,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: 3,
                Direction.B: 1,
                Direction.R: 4,
                Direction.L: None,
            },
        ),
        Object(
            1,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: 0,
                Direction.B: None,
                Direction.R: None,
                Direction.L: None,
            },
        ),
        Object(
            3,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: None,
                Direction.B: 0,
                Direction.R: None,
                Direction.L: None,
            },
        ),
    ]
    robot.detector = ArucoDetector(
        aruco_dict_type=cv2.aruco.DICT_4X4_50,
        marker_size=0.15,
        calibration_file="./calibration_chessboard.yaml",
        camera_index=0,
        z_offset=-28,
        marker_list=markers,
    )
    robot.detector.routes = [1, 0, 4]
    robot_detector_thread = threading.Thread(target=robot.detector.Start)
    # robot.broadcaster = Broadcaster()
    robot_detector_thread.start()
    print("Detector Started")
    # time.sleep(1)
    # asyncio.run(robot.broadcaster.Start(robot.detector))
    # print("Broadcaster started")

    # try:
    #     while True:
    #         time.sleep(1)
    # except (KeyboardInterrupt, asyncio.CancelledError):
    #     print("Main thread interrupted")
    #     robot.broadcaster.Stop()
    #     robot_detector_thread.join()
