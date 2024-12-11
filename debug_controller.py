import controller


class Controller(controller.Controller):
    def Forward(self):
        print("Forward...")

    def Turn180(self):
        self.TurnLeft()
        self.TurnLeft()

    def TurnLeft(self):
        print("Left...")

    def TurnRight(self):
        print("Right...")

    def Stop(self):
        print("Stop...")

    def Disconnect(self):
        self.Stop()
        print("Disconnect...")
