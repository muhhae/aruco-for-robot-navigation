import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import math

aruco_size = 0.05

camera_calibration_filename = "./calibration_chessboard.yaml"


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def main():
    cv_file = cv2.FileStorage(camera_calibration_filename, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode("K").mat()
    dist = cv_file.getNode("D").mat()
    cv_file.release()

    dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    param = cv2.aruco.DetectorParameters()

    cap = cv2.VideoCapture(0)

    while 1:
        ret, frame = cap.read()
        if ret is None:
            return
        undistorted = cv2.undistort(frame, mtx, dist)
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            undistorted,
            dict,
            parameters=param,
        )
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners, aruco_size, mtx, dist
            )
            for i, marker_id in enumerate(marker_ids):
                transform_translation_x = tvecs[i][0][0]
                transform_translation_y = tvecs[i][0][1]
                transform_translation_z = tvecs[i][0][2]

                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = Rotation.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()

                transform_rotation_x = quat[0]
                transform_rotation_y = quat[1]
                transform_rotation_z = quat[2]
                transform_rotation_w = quat[3]

                roll_x, pitch_y, yaw_z = quaternion_to_euler(
                    transform_rotation_x,
                    transform_rotation_y,
                    transform_rotation_z,
                    transform_rotation_w,
                )

                roll_x = math.degrees(roll_x)
                pitch_y = math.degrees(pitch_y)
                yaw_z = math.degrees(yaw_z)
                print("transform_translation_x: {}".format(transform_translation_x))
                print("transform_translation_y: {}".format(transform_translation_y))
                print("transform_translation_z: {}".format(transform_translation_z))
                print("roll_x: {}".format(roll_x))
                print("pitch_y: {}".format(pitch_y))
                print("yaw_z: {}".format(yaw_z))
                print()

                cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)

        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Close down the video stream
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print(__doc__)
    main()
