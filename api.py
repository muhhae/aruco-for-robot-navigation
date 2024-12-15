from fastapi import FastAPI
from aruco_detector import ArucoDetector, RobotState
import uvicorn
from pydantic import BaseModel
from util import JSONToMarkers


class RobotAPI:
    detector: ArucoDetector
    app: FastAPI

    def __init__(self, detector: ArucoDetector):
        self.app = FastAPI()
        self.detector = detector
        self.SetupRoutes()

    def SetupRoutes(self):
        @self.app.get("/")
        def root():
            return {"message": "This API is working properly"}

        class change_state_r(BaseModel):
            state: str

        @self.app.post("/state/")
        def change_state(r: change_state_r):
            if r.state == "stop":
                self.detector.state = RobotState.STOP
                return {"message": "Robot state changed to STOP"}
            elif r.state == "running":
                self.detector.state = RobotState.RUNNING
                return {"message": "Robot state changed to RUNNING"}
            else:
                return {"message": "state should be stop or running"}

        @self.app.get("/position/")
        def get_position():
            return {"position": self.detector.current_position}

        @self.app.get("/state/")
        def get_state():
            return {
                "robot_state": "running"
                if self.detector.state == RobotState.RUNNING
                else "stop"
            }

        class set_routes_r(BaseModel):
            routes: list[int]

        @self.app.post("/routes/")
        def set_routes(r: set_routes_r):
            if self.detector.state != RobotState.STOP:
                return {"message": "Robot should be stopped first"}
            self.detector.routes = r.routes
            return {
                "messages": "Routes changed",
                "routes": self.detector.routes,
            }

        @self.app.get("/routes/")
        def get_routes():
            return {
                "routes": self.detector.routes,
            }

        @self.app.get("/maps/")
        def get_maps():
            return {
                "maps": self.detector.marker_list,
            }

        class set_maps_r(BaseModel):
            maps: list[list[str | None]]

        @self.app.post("/maps/")
        def set_maps(r: set_maps_r):
            if self.detector.state != RobotState.STOP:
                return {"message": "Robot should be stopped first"}
            markers = JSONToMarkers(r.maps)
            self.detector.marker_list = markers
            return {
                "message": "maps changed",
                "maps": self.detector.marker_list,
            }

    def Start(self):
        uvicorn.run(self.app, host="127.0.0.1", port=3000)
