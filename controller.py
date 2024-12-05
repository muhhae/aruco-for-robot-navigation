from wheel import MotorDriver


class RobotControl:
    """
    motor PWM PINS
    motor ENB PINS
    as dict

    {
        "pwm1" : 12,
        "pwm2" : 18,
        "enb1" : 14,
        "enb2" : 15,
    }


    """

    def __init__(self, left, right):
        self.left_wheel = MotorDriver(
            left["pwm1"], left["pwm2"], left["enb1"], left["enb2"]
        )
        self.right_wheel = MotorDriver(
            right["pwm1"], right["pwm2"], right["enb1"], right["enb2"]
        )

    def robot_forward(self, speed=25):
        self.left_wheel.move_reverse(speed=speed)
        self.right_wheel.move_forward(speed=speed)

    def robot_backward(self, speed=25):
        self.left_wheel.move_forward(speed=speed)
        self.right_wheel.move_reverse(speed=speed)

    def robot_turn_right(self, speed=25):
        self.left_wheel.move_reverse(speed=speed)
        self.right_wheel.move_forward(speed=speed / 2)

    def robot_turn_left(self, speed=25):
        self.left_wheel.move_forward(speed=speed)
        self.right_wheel.move_reverse(speed=speed / 2)

    def robot_pivot_right(self, speed=25):
        self.left_wheel.move_reverse(speed=speed)
        self.right_wheel.move_reverse(speed=speed)

    def robot_pivot_left(self, speed=25):
        self.left_wheel.move_forward(speed=speed)
        self.right_wheel.move_forward(speed=speed)

    def robot_stop(self):
        self.left_wheel.stop_moving()
        self.right_wheel.stop_moving()

    def disconnect(self):
        self.left_wheel.disconnect()
        self.right_wheel.disconnect()
