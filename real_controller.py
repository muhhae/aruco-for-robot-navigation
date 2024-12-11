import controller
from ArucoRobotControl.robot import RobotControl as RC


class Controller(controller.Controller):
    rc: RC

    def __init__(self):
        self.rc = RC(
            {
                "pwm1": 12,
                "pwm2": 13,
                "enb1": 5,
                "enb2": 6,
            },
            {
                "pwm1": 18,
                "pwm2": 19,
                "enb1": 20,
                "enb2": 21,
            },
        )

    def Forward(self):
        self.rc.robot_forward(25)

    def Turn180(self):
        self.TurnLeft()
        self.TurnLeft()

    def TurnLeft(self):
        self.rc.robot_pivot_left()

    def TurnRight(self):
        self.rc.robot_pivot_right()

    def Stop(self):
        self.rc.robot_stop()

    def Disconnect(self):
        self.Stop()
        self.rc.disconnect()
