import signal
import cv2
from aruco_detector import Object, Direction, ObjectType, ArucoDetector

from broadcaster import Broadcaster
import asyncio
import os
import threading

is_dev = os.getenv("DEV", "false").lower() in ("true", "1", "yes")
if not is_dev:
    from real_controller import Controller
else:
    from debug_controller import Controller


class Robot:
    detector: ArucoDetector
    broadcaster: Broadcaster
    controller: Controller
    thread: threading.Thread

    def __init__(self):
        pass

    async def Start(self):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        try:
            self.detector.Start()
            # await self.broadcaster.Start(self.detector)
        except (KeyboardInterrupt, asyncio.CancelledError):
            # self.broadcaster.Stop()
            self.detector.Stop()

    def Stop():
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
        controller=Controller(),
    )
    robot.detector.routes = [1, 0, 4]
    robot.broadcaster = Broadcaster()
    asyncio.run(robot.Start())
