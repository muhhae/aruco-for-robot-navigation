import signal
import cv2
from aruco_detector import Object, Direction, ArucoDetector
from util import JSONToMarkers

from broadcaster import Broadcaster
import asyncio
import os
import threading
from api import RobotAPI

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
    markers = JSONToMarkers(
        [
            [None, "A6", None],
            [None, "A5", None],
            ["A4", "A2", "A3"],
            [None, "A1", None],
            [None, "A0", None],
        ]
    )
    robot.detector = ArucoDetector(
        aruco_dict_type=cv2.aruco.DICT_4X4_50,
        marker_size=0.15,
        calibration_file="./calibration_chessboard.yaml",
        camera_index=0,
        z_offset=-28,
        marker_list=markers,
        controller=Controller(),
    )
    robot.detector.routes = [0, 1, 2, 3]
    robot.broadcaster = Broadcaster()
    asyncio.run(robot.Start())
    api = RobotAPI(robot.detector)
    api.Start()
    print("not blocked")
