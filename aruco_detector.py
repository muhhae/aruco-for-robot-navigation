import cv2
import os
from enum import Enum
import numpy as np
from typing import Dict
from controller import Controller
import threading

os.environ["QT_QPA_PLATFORM"] = "xcb"
camera_calibration_filename = "./calibration_chessboard.yaml"


def rvec_to_euler_angles(rvec):
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)

    singular = sy < 1e-6
    if not singular:
        pitch = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        yaw = np.arctan2(-rotation_matrix[2, 0], sy)
        roll = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        pitch = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        yaw = np.arctan2(-rotation_matrix[2, 0], sy)
        roll = 0

    return np.degrees([pitch, yaw, roll])


class ArucoTransform:
    x: float
    y: float
    z: float

    x_rot: float
    y_rot: float
    z_rot: float

    id: int

    def __init__(self, id, x, y, z, x_rot, y_rot, z_rot):
        self.x = x
        self.y = y
        self.z = z

        self.x_rot = x_rot
        self.y_rot = y_rot
        self.z_rot = z_rot

        self.id = id


class Direction(Enum):
    T = 0
    B = 1
    L = 2
    R = 3


class RobotState(Enum):
    READY = 0
    RUNNING = 1
    STOP = 2
    FINISH = 3


class ObjectType(Enum):
    ARUCO_MARKER = 0
    ITEM = 1


class Object:
    Type: ObjectType

    id: int
    neighbour: Dict[Direction, int] = {}

    def __init__(self, id, ObjectType, neighbour):
        self.id = id
        self.Type = ObjectType
        self.neighbour = neighbour

    def __eq__(self, other) -> bool:
        return self.id == other.id and self.Type == other.Type


class ArucoDetector:
    thread: threading.Thread = None
    current_move = None
    past_move = None
    controller: Controller
    frame: cv2.UMat = None
    aruco_dict: cv2.aruco.Dictionary
    marker_size: float
    camera_matrix: cv2.typing.MatLike
    distortion_coeff: cv2.typing.MatLike
    camera: cv2.VideoCapture
    z_offset: float
    marker_list: list[Object]
    current_position: Object
    state: RobotState
    routes: list[Object]
    orientation: Direction
    orientation_dict = {
        Direction.T: {
            Direction.T: Direction.T,
            Direction.B: Direction.B,
            Direction.L: Direction.L,
            Direction.R: Direction.R,
        },
        Direction.B: {
            Direction.T: Direction.B,
            Direction.B: Direction.T,
            Direction.L: Direction.R,
            Direction.R: Direction.L,
        },
        Direction.L: {
            Direction.T: Direction.R,
            Direction.B: Direction.L,
            Direction.L: Direction.B,
            Direction.R: Direction.T,
        },
        Direction.R: {
            Direction.T: Direction.L,
            Direction.B: Direction.R,
            Direction.L: Direction.T,
            Direction.R: Direction.B,
        },
    }

    def __init__(
        self,
        marker_size: float,
        calibration_file: str,
        aruco_dict_type: int,
        camera_index: int,
        z_offset: float,
        marker_list: list[Object],
        controller: Controller,
    ):
        fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("K").mat()
        self.distortion_coeff = fs.getNode("D").mat()
        fs.release()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.camera = cv2.VideoCapture(camera_index)
        self.marker_size = marker_size
        self.z_offset = z_offset
        self.marker_list = marker_list
        self.state = RobotState.READY
        self.current_position = None
        self.controller = controller

    def GetPosition(
        self, aruco_transforms: list[ArucoTransform]
    ) -> (int, int, Direction):
        nearest = aruco_transforms[0]

        for e in aruco_transforms:
            if e.z < nearest.z:
                nearest = e

        direction = None
        if abs(nearest.z_rot - 180) < 20 or abs(nearest.z_rot - (-180)) < 20:
            direction = Direction.T
            self.orientation = Direction.B
        elif abs(nearest.z_rot - 0) < 20:
            direction = Direction.B
            self.orientation = Direction.T
        elif abs(nearest.z_rot - 90) < 20:
            direction = Direction.R
            self.orientation = Direction.L
        elif abs(nearest.z_rot - (-90)) < 20:
            direction = Direction.L
            self.orientation = Direction.R
        return nearest.id, nearest.z, direction

    def CurrentTask(self, distance: float):
        if self.current_position is None:
            return

        self.controller.Forward()
        n = distance - 30
        if abs(n) < 1:
            print("Exactly at", self.current_position.id)
            print("Stop..")
            next_id_index = self.routes.index(self.current_position.id) + 1
            next_id = self.routes[next_id_index]
            T = self.orientation_dict[Direction.T][self.orientation]
            B = self.orientation_dict[Direction.B][self.orientation]
            L = self.orientation_dict[Direction.L][self.orientation]
            R = self.orientation_dict[Direction.R][self.orientation]
            if self.current_position.neighbour[T] == next_id:
                self.current_move = "Maju"
            elif self.current_position.neighbour[L] == next_id:
                self.controller.TurnLeft()
                self.current_move = "Kiri"
            elif self.current_position.neighbour[R] == next_id:
                self.controller.TurnRight()
                self.current_move = "Kanan"
            elif self.current_position.neighbour[B] == next_id:
                self.controller.Turn180()
                self.current_move = "Mundur"

    def ProcessArucoTransform(self, id, dis, dir):
        aruco_marker = None
        self.current_position = None
        for e in self.marker_list:
            if e.id == id and e.Type == ObjectType.ARUCO_MARKER:
                aruco_marker = e
                break
        if aruco_marker is not None and dir is not None:
            current_id = aruco_marker.neighbour[dir]
            print("id ", id, "dis ", dis, "dir ", dir)
            print("current ", current_id)
            for marker in self.marker_list:
                if marker.id == current_id:
                    self.current_position = marker
                    if self.current_position.id == self.routes[-1]:
                        self.state == RobotState.STOP

        return dis

    def Stop(self):
        self.thread.join()
        self.thread = None

    def Start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self.Run)
            self.thread.start()

    def Run(self):
        while 1:
            ret, frame = self.camera.read()
            if not ret or frame is None:
                print("Something wrong with the camera")
                break

            aruco_transforms = self.Detect(frame)
            if aruco_transforms is not None:
                id, dis, dir = self.GetPosition(aruco_transforms)
                self.ProcessArucoTransform(id, dis, dir)
                self.CurrentTask(dis)
            # if self.current_move is not None and self.past_move != self.current_move:
            #     if self.current_move == "Maju":
            #         self.controller.robot_forward()
            #     elif self.current_move == "Mundur":
            #         self.controller.robot_backward()
            #     elif self.current_move == "Kanan":
            #         self.controller.robot_turn_right()
            #     elif self.current_move == "Kiri":
            #         self.controller.robot_pivot_left()
            self.past_move = self.current_move
            self.frame = frame.copy()
        #     cv2.imshow("copy: ", self.frame)
        #     cv2.imshow("original: ", frame)
        #     if cv2.waitKey(1) & 0xFF == ord("q"):
        #         break
        # cv2.destroyAllWindows()

    def Detect(self, frame: cv2.UMat) -> list[ArucoTransform] | None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        undistorted_frame = cv2.undistort(
            gray, self.camera_matrix, self.distortion_coeff
        )
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            undistorted_frame, self.aruco_dict, parameters=self.detector_parameters
        )
        cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.distortion_coeff
        )
        if marker_ids is None:
            return None

        aruco_transforms = []
        for i, marker_id in enumerate(marker_ids):
            x = tvecs[i][0][0] * 100
            y = tvecs[i][0][1] * 100
            z = (tvecs[i][0][2] * 100) + self.z_offset

            rot = rvec_to_euler_angles(rvecs[i])

            aruco_transforms.append(
                ArucoTransform(marker_id[0], x, y, z, rot[0], rot[1], rot[2])
            )

            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.distortion_coeff,
                rvecs[i],
                tvecs[i],
                0.05,
            )
        return aruco_transforms


def main():
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

    detector = ArucoDetector(
        aruco_dict_type=cv2.aruco.DICT_4X4_50,
        marker_size=0.10,
        calibration_file="./calibration_chessboard.yaml",
        camera_index=0,
        z_offset=-28,
        marker_list=markers,
    )
    detector.routes = [0, 1, 3, 5, 6]
    detector.Start()


if __name__ == "__main__":
    main()
