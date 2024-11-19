import cv2
import os

os.environ["QT_QPA_PLATFORM"] = "xcb"
camera_calibration_filename = "./calibration_chessboard.yaml"

warehouse_map = [[0, 1, 2, 3], [4, 5, 6, 7]]
dist_between = 0.3


class ArucoTransform:
    x: float
    y: float
    z: float
    id: int

    def __init__(self, id, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.id = id


class ArucoDetector:
    aruco_dict: cv2.aruco.Dictionary
    marker_size: float
    camera_matrix: cv2.typing.MatLike
    distortion_coeff: cv2.typing.MatLike
    camera: cv2.VideoCapture
    z_offset: float

    def __init__(
        self,
        marker_size: float,
        calibration_file: str,
        aruco_dict_type: int,
        camera_index: int,
        z_offset: float,
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

    def GetPosition(self, aruco_transforms: list[ArucoTransform]) -> (int, int):
        nearest = aruco_transforms[0]
        second_nearest = aruco_transforms[0]

        for aruco_transform in aruco_transforms[1:]:
            if aruco_transform.z < nearest.z:
                second_nearest = nearest
                nearest = aruco_transform
            elif aruco_transform.z < second_nearest.z:
                second_nearest = aruco_transform.z

        nearest_column = -1
        nearest_row = -1
        second_nearest_column = -1
        second_nearest_row = -1

        for i, row in enumerate(warehouse_map):
            if nearest_column != -1 and second_nearest_column != -1:
                break
            if nearest_column == -1:
                nearest_row = row
                nearest_column = row.index(nearest.id)
            if second_nearest_column == -1:
                second_nearest_row = row
                second_nearest_column = row.index(second_nearest.id)

        row = 2 * nearest_row - second_nearest_row
        column = 2 * nearest_column - second_nearest_column

        return row, column

    def Run(self):
        while 1:
            ret, frame = self.camera.read()
            if ret is None:
                break

            aruco_transforms = self.Detect(frame)
            if aruco_transforms is not None:
                self.GetPosition(aruco_transforms)
                for aruco_transform in aruco_transforms:
                    x = aruco_transform.x * 100
                    y = aruco_transform.y * 100
                    z = aruco_transform.z * 100 + self.z_offset

                    print("id ", aruco_transform.id)
                    print(f"{x:>5.0f}cm {y:>5.0f}cm {z:>5.0f}cm")

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
            x = tvecs[i][0][0]
            y = tvecs[i][0][1]
            z = tvecs[i][0][2]
            aruco_transforms.append(ArucoTransform(marker_id, x, y, z))

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
    detector = ArucoDetector(
        aruco_dict_type=cv2.aruco.DICT_4X4_50,
        marker_size=0.10,
        calibration_file="./calibration_chessboard.yaml",
        camera_index=0,
        z_offset=-28,
    )
    detector.Run()


if __name__ == "__main__":
    main()
