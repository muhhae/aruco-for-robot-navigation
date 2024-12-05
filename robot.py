import time
import cv2
from aruco_detector import Object, Direction, ObjectType, ArucoDetector
from broadcaster import Broadcaster
import threading
import asyncio


class Robot:
    detector: ArucoDetector
    broadcaster: Broadcaster

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
                Direction.T: 1,
                Direction.B: None,
                Direction.R: None,
                Direction.L: None,
            },
        ),
        Object(
            1,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: 2,
                Direction.B: 0,
                Direction.R: 3,
                Direction.L: None,
            },
        ),
        Object(
            2,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: None,
                Direction.B: 1,
                Direction.R: None,
                Direction.L: None,
            },
        ),
        Object(
            3,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: None,
                Direction.B: None,
                Direction.R: 5,
                Direction.L: 1,
            },
        ),
        Object(
            4,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: None,
                Direction.B: 6,
                Direction.R: None,
                Direction.L: None,
            },
        ),
        Object(
            5,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: 6,
                Direction.B: None,
                Direction.R: 7,
                Direction.L: 3,
            },
        ),
        Object(
            6,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: 4,
                Direction.B: 5,
                Direction.R: None,
                Direction.L: None,
            },
        ),
        Object(
            7,
            ObjectType.ARUCO_MARKER,
            {
                Direction.T: None,
                Direction.B: None,
                Direction.R: None,
                Direction.L: 5,
            },
        ),
    ]
    robot.detector = ArucoDetector(
        aruco_dict_type=cv2.aruco.DICT_4X4_50,
        marker_size=0.10,
        calibration_file="./calibration_chessboard.yaml",
        camera_index=0,
        z_offset=-28,
        marker_list=markers,
    )
    robot.detector.routes = [0, 1, 3, 5, 6]
    robot_detector_thread = threading.Thread(target=robot.detector.Start)
    robot.broadcaster = Broadcaster()

    robot_detector_thread.start()
    asyncio.run(robot.broadcaster.Start(robot.detector))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Main thread interrupted")
        robot_detector_thread.join()
