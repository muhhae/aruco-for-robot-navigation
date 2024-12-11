import asyncio
import websockets
import cv2
import base64
from httpserver import HttpServer
import signal
import json
from aruco_detector import ArucoDetector


class Broadcaster:
    clients = set()
    httpserver = HttpServer(8088)

    def __init__(self):
        pass

    async def Start(self, detector: ArucoDetector):
        self.httpserver.start()
        async with websockets.serve(self.handler, "", 8089):
            await self.broadcast(detector)

    def Stop(self):
        self.httpserver.stop()

    async def handler(self, websocket):
        print("Handled")
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.remove(websocket)
        print("clients", self.clients)

    async def send(self, websocket, base64Frame: str):
        try:
            # await websocket.send(base64Frame)
            payload = {}
            payload["frame"] = base64Frame
            await websocket.send(json.dumps(payload))
        except (websockets.ConnectionClosed, AssertionError):
            pass

    async def broadcast(self, detector: ArucoDetector):
        while True:
            frame = ArucoDetector.frame
            if frame is None:
                continue
            ret, encoded = cv2.imencode(".png", frame)
            if not ret:
                continue
            base64Frame = base64.b64encode(encoded).decode("ascii")
            for websocket in self.clients:
                await self.send(websocket, base64Frame)
            await asyncio.sleep(0.04)


async def main():
    # This restores the default Ctrl+C signal handler, which just kills the process
    # This is a workaround with the problem of asyncio cleaning up the websockets with Ctrl+C
    # Remove this line and it will shutdown gracefully if no client is connected but will not once a connection is made
    # TODO find a better way to gracefully clean-up and shutdown
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    broadcaster = Broadcaster()
    httpserver = HttpServer(8088)

    try:
        httpserver.start()
        async with websockets.serve(broadcaster.handler, "", 8089):
            await broadcaster.broadcast()
    except (KeyboardInterrupt, asyncio.CancelledError):
        httpserver.stop()


if __name__ == "__main__":
    asyncio.run(main())
