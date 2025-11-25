import cv2
import numpy as np
import asyncio
import websockets
import os
from pathlib import Path

PANORAMA_SAVE_PATH = str(Path.home() / "sophia_ws/panorama_output.jpg")
SERVER_URI = "ws://192.168.1.23:8765"

async def receive_stream():
    async with websockets.connect(SERVER_URI) as websocket:
        print("Connected to server")

        while True:
            try:
                msg = await websocket.recv()

                # Handle panoramic image
                if isinstance(msg, bytes) and msg.startswith(b"PANORAMA_IMAGE:"):
                    print("Received panorama image.")
                    image_data = msg[len(b"PANORAMA_IMAGE:"):]
                    with open(PANORAMA_SAVE_PATH, "wb") as f:
                        f.write(image_data)
                    print(f"Panorama saved to {PANORAMA_SAVE_PATH}")
                    continue

                # Handle video frame size (sent as string)
                if isinstance(msg, str) and msg.isdigit():
                    frame_size = int(msg)
                    frame_data = b""

                    while len(frame_data) < frame_size:
                        chunk = await websocket.recv()
                        if isinstance(chunk, bytes):
                            frame_data += chunk

                    np_data = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

                    if frame is not None:
                        cv2.imshow("Video Feed", frame)
                    else:
                        print("No video feed available")

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key in map(ord, '01234'):
                        print(chr(key))
                        await websocket.send(chr(key))
            except websockets.ConnectionClosed:
                print("Connection closed.")
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(receive_stream())
