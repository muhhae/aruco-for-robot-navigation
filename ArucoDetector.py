import cv2
import os
from enum import Enum
import numpy as np
from typing import Dict

os.environ["QT_QPA_PLATFORM"] = "xcb"
camera_calibration_filename = "./calibration_chessboard.yaml"


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


class ArucoMarker:
    id: int
    neighbour: Dict[Direction, int] = {}

    def __init__(self, id):
        self.id = id


def rvec_to_euler_angles(rvec):
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)

    singular = sy < 1e-6  # Check for singularity
    if not singular:
        pitch = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        yaw = np.arctan2(-rotation_matrix[2, 0], sy)
        roll = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        pitch = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        yaw = np.arctan2(-rotation_matrix[2, 0], sy)
        roll = 0

    return np.degrees([pitch, yaw, roll])  # Convert to degrees


class ArucoDetector:
    aruco_dict: cv2.aruco.Dictionary
    marker_size: float
    camera_matrix: cv2.typing.MatLike
    distortion_coeff: cv2.typing.MatLike
    camera: cv2.VideoCapture
    z_offset: float
    marker_list: list[ArucoMarker]

    def __init__(
        self,
        marker_size: float,
        calibration_file: str,
        aruco_dict_type: int,
        camera_index: int,
        z_offset: float,
        marker_list: list[ArucoMarker],
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

    def GetPosition(
        self, aruco_transforms: list[ArucoTransform]
    ) -> (int, int, Direction):
        nearest = aruco_transforms[0]

        for e in aruco_transforms:
            if e.z > nearest.z:
                nearest = e

        direction = None
        if abs(nearest.z_rot - 180) < 20 or abs(nearest.z_rot - (-180)) < 20:
            direction = Direction.T
        elif abs(nearest.z_rot - 0) < 20:
            direction = Direction.B
        elif abs(nearest.z_rot - 90) < 20:
            direction = Direction.R
        elif abs(nearest.z_rot - (-90)) < 20:
            direction = Direction.L
        return nearest.id, nearest.z, direction

    def Run(self):
        while 1:
            ret, frame = self.camera.read()
            if ret is None:
                break

            aruco_transforms = self.Detect(frame)
            if aruco_transforms is not None:
                id, dis, dir = self.GetPosition(aruco_transforms)
                aruco_marker = None
                for e in self.marker_list:
                    if e.id == id:
                        aruco_marker = e
                        break
                if aruco_marker is not None and dir is not None:
                    current_id = aruco_marker.neighbour[dir]

                    print("id ", id, "dis ", dis, "dir ", dir)
                    print("current ", current_id)

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cv2.destroyAllWindows()

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
    marker_0 = ArucoMarker(0)
    marker_0.neighbour = {
        Direction.T: 4,
        Direction.B: 1,
        Direction.R: 2,
        Direction.L: 3,
    }

    marker_list = []
    marker_list.append(marker_0)

    detector = ArucoDetector(
        aruco_dict_type=cv2.aruco.DICT_4X4_50,
        marker_size=0.10,
        calibration_file="./calibration_chessboard.yaml",
        camera_index=0,
        z_offset=-28,
        marker_list=marker_list,
    )
    detector.Run()


if __name__ == "__main__":
    main()
