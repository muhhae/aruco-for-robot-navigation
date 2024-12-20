import controller
from ArucoRobotControl.robot import RobotControl as RC
from time import sleep


class Controller(controller.Controller):
    rc: RC

    def __init__(self):
        self.rc = RC(
            {
                "pwm1": 12,
                "pwm2": 13,
                "enb1": 5,
                "enb2": 6,
                "enc1": 23,
                "enc2": 24,
            },
            {
                "pwm1": 18,
                "pwm2": 19,
                "enb1": 20,
                "enc1": 5,
                "enc2": 6,
                "enb2": 21,
            },
        )

    def Forward(self):
        self.rc.robot_forward()

    def Turn180(self):
        self.TurnLeft()
        self.TurnLeft()
        print("Stop 180")

    def TurnLeft(self):
        self.rc.robot_pivot_left()
        print("in left pivot motion...")
        sleep(0.4)
        self.rc.stop()

    def TurnRight(self):
        self.rc.robot_pivot_right()
        print("in right pivot motion...")
        sleep(0.4)
        self.rc.stop()

    def PivotLeft(self):
        self.rc.robot_pivot_left(10)

    def PivotRight(self):
        self.rc.robot_pivot_right(10)

    def Stop(self):
        self.rc.stop()

    def Disconnect(self):
        self.Stop()
        self.rc.disconnect()
