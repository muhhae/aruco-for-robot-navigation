import controller
from ArucoRobotControl.robot import RobotControl as RC


class Pin:
    pwm_1: int
    pwm_2: int
    enb_1: int
    enb_2: int


class Controller(controller.Controller):
    rc: RC

    def __init__(self, left: Pin, right: Pin):
        self.rc = RC(
            {
                "pwm1": left.pwm_1,
                "pwm2": left.pwm_2,
                "enb1": left.enb_1,
                "enb2": left.enb_2,
            },
            {
                "pwm1": right.pwm_1,
                "pwm2": right.pwm_2,
                "enb1": right.enb_1,
                "enb2": right.enb_2,
            },
        )

    def Forward(self):
        self.rc.robot_forward()

    def Turn180(self):
        self.TurnLeft()
        self.TurnLeft()

    def TurnLeft(self):
        self.rc.robot_pivot_left()

    def TurnRight(self):
        self.rc.robot_pivot_right()

    def Stop(self):
        self.rc.robot_stop()
