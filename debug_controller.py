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

    def PivotLeft(self):
        print("Pivot Left...")

    def PivotRight(self):
        print("Pivot Rigth...")

    def Disconnect(self):
        self.Stop()
        print("Disconnect...")
