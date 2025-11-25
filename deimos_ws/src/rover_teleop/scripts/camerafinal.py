#!/usr/bin/env python3
import asyncio
import threading
import time
import os
import cv2
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16MultiArray
import websockets
import numpy as np

# === CONFIGURATION ===
PANORAMA_CAMERA_INDEX = 10
FEED_INDICES = [PANORAMA_CAMERA_INDEX, 8, 0, 6]
QUALITY = 40
DESIRED_FPS = 20
WIDTH = 640
HEIGHT = 480
JOYSTICK_TRIGGER_BUTTON = 9
PANORAMA_DIR = '/root/sophia_ws/panorama_images'


# === CAMERA MANAGER ===
class CameraManager:
    def __init__(self, indices, width, height, fps):
        self.feed_indices = indices
        self.current_index = 0
        self.width = width
        self.height = height
        self.fps = fps
        self.lock = asyncio.Lock()
        self.cap = None
        self._open_camera(self.feed_indices[self.current_index])

    def _open_camera(self, index):
        if self.cap:
            self.cap.release()
        self.cap = cv2.VideoCapture(f"/dev/video{index}", cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open /dev/video{index}")
        print(f"Opened camera /dev/video{index}")

    async def switch_feed(self, new_index):
        async with self.lock:
            if 0 <= new_index < len(self.feed_indices):
                self.current_index = new_index
                self._open_camera(self.feed_indices[new_index])
                print(f"Switched to camera index {new_index}")

    async def read_frame(self):
        async with self.lock:
            ret, frame = self.cap.read()
            if not ret:
                raise RuntimeError("Failed to read frame")
            return frame

    def sync_read(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame synchronously")
        return frame

    def exclusive_feed(self, override_index):
        class ExclusiveContext:
            def __init__(inner_self):
                inner_self.outer = self

            async def __aenter__(inner_self):
                await self.lock.acquire()
                self._open_camera(override_index)

            async def __aexit__(inner_self, exc_type, exc_val, exc_tb):
                self._open_camera(self.feed_indices[self.current_index])
                self.lock.release()

        return ExclusiveContext()

    def release(self):
        if self.cap:
            self.cap.release()


# === ROS PANORAMA NODE ===
class JoyPanoramaTrigger:
    def __init__(self, camera_manager, loop, websocket=None):
        self.camera_manager = camera_manager
        self.loop = loop
        self.websocket = websocket

        rospy.init_node('joy_panorama_trigger', disable_signals=True)
        self.servo_pub = rospy.Publisher('/camera_servo_angle', UInt16MultiArray, queue_size=10)
        rospy.Subscriber('/j1', Joy, self.joy_callback)

        rospy.loginfo("Panorama trigger initialized")
        os.makedirs(PANORAMA_DIR, exist_ok=True)
        self.angles = [0, 34, 67, 100, 135]

        self.in_progress = False
        self.current_angle_index = 0
        self.image_paths = []

    def joy_callback(self, msg):
        if len(msg.buttons) > JOYSTICK_TRIGGER_BUTTON and msg.buttons[JOYSTICK_TRIGGER_BUTTON] == 1:
            if not self.in_progress:
                rospy.loginfo("Panorama triggered")
                self.in_progress = True
                self.current_angle_index = 0
                self.image_paths = []
                asyncio.run_coroutine_threadsafe(self.process_next_angle(), self.loop)
            else:
                rospy.loginfo("Continuing to next angle...")
                asyncio.run_coroutine_threadsafe(self.process_next_angle(), self.loop)

    async def process_next_angle(self):
        if self.current_angle_index < len(self.angles):
            angle = self.angles[self.current_angle_index]
            rospy.loginfo(f"Publishing angle {angle}...")
            self.servo_pub.publish(UInt16MultiArray(data=[angle]))

            await asyncio.sleep(1.5)  # Let servo settle
            rospy.loginfo(f"Capturing image at angle {angle}...")

            async with self.camera_manager.exclusive_feed(PANORAMA_CAMERA_INDEX):
                frame = self.camera_manager.sync_read()
                path = os.path.join(PANORAMA_DIR, f'image_{self.current_angle_index + 1}.jpg')
                cv2.imwrite(path, frame)
                rospy.loginfo(f"Captured: {path}")
                self.image_paths.append(path)

            self.current_angle_index += 1
            rospy.loginfo("Waiting for next button press to continue...")
        else:
            rospy.loginfo("All angles captured. Stitching images...")
            self.in_progress = False
            self.stitch_images(self.image_paths)

    def stitch_images(self, paths):
        rospy.loginfo("Stitching images...")
        images = [cv2.imread(p) for p in paths]
        if any(img is None for img in images):
            rospy.logerr("Missing image in stitching")
            return

        stitcher = cv2.Stitcher_create()
        status, pano = stitcher.stitch(images)
        if status == cv2.Stitcher_OK:
            try:
                heading = float(input("Enter GPS heading for this panorama (0-360): "))
            except ValueError:
                rospy.logwarn("Invalid heading. Skipping cardinal direction labels.")
                heading = None

            if heading is not None:
                self.label_cardinal_directions(pano, heading)

            self.annotate_scale_with_rover(pano)

            output_path = os.path.join(PANORAMA_DIR, "panorama_output.jpg")
            cv2.imwrite(output_path, pano)
            rospy.loginfo(f"Saved panorama to {output_path}")

            if self.websocket:
                _, buffer = cv2.imencode('.jpg', pano)
                try:
                    asyncio.run_coroutine_threadsafe(
                        self.websocket.send(b"PANORAMA_IMAGE:" + buffer.tobytes()), self.loop
                    )
                except Exception as e:
                    rospy.logerr(f"Failed to send panorama over WebSocket: {e}")

            default_angle = 67
            rospy.loginfo(f"Resetting servo to default angle: {default_angle}")
            self.servo_pub.publish(UInt16MultiArray(data=[default_angle]))
        else:
            self.log_stitch_error(status)


    def log_stitch_error(self, status):
        messages = {
            1: "Need more images",
            2: "Homography failed",
            3: "Camera params adjustment failed"
        }
        rospy.logerr(f"Stitching failed: {messages.get(status, 'Unknown error')}")

    def label_cardinal_directions(self, image, heading_deg):
        def normalize_angle(angle):
            return angle % 360

        fov_deg = self.angles[-1] - self.angles[0]
        img_width = image.shape[1]
        deg_per_px = fov_deg / img_width

        # Cardinal directions and their global angles
        directions = {
            "N": 0,
            "E": 90,
            "S": 180,
            "W": 270
        }

        label_y = 60  # Y position for text
        font_scale = 1
        color = (0, 0, 255)
        thickness = 2

        for label, absolute_angle in directions.items():
            # Calculate angle relative to panorama center
            relative_angle = normalize_angle(absolute_angle - heading_deg + fov_deg / 2)
            pixel_x = int(relative_angle / deg_per_px)

            # Draw only if the label falls within the stitched image
            if 0 <= pixel_x < img_width:
                cv2.putText(
                    image,
                    label,
                    (pixel_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    color,
                    thickness,
                    cv2.LINE_AA
                )
    
    def annotate_scale_with_rover(self, image, known_rover_width_m=1.2):
        height, width = image.shape[:2]
        center_x = width // 2

        # Extract narrow vertical ROI centered in image
        roi_width = 200
        roi = image[:, center_x - roi_width // 2 : center_x + roi_width // 2]

        # Convert to HSV and find low-saturation (gray) pixels
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        gray_mask = saturation < 50  # Threshold for grayness

        # Horizontal projection (sum over height)
        gray_projection = np.sum(gray_mask, axis=0)
        threshold = np.max(gray_projection) * 0.3  # Adjustable

        edges = np.where(gray_projection > threshold)[0]
        if len(edges) < 2:
            rospy.logwarn("Could not reliably detect rover edges for scale annotation")
            return  # Skip annotation

        # Convert edges back to full image x-coordinates
        left_edge = edges[0] + center_x - roi_width // 2
        right_edge = edges[-1] + center_x - roi_width // 2
        rover_width_px = right_edge - left_edge

        # Draw line and label
        y_pos = height - 40
        cv2.line(image, (left_edge, y_pos), (right_edge, y_pos), (255, 255, 255), 2)

        label = f"Rover ~{known_rover_width_m:.1f} m"
        cv2.putText(
            image,
            label,
            (left_edge, y_pos - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        # Optionally, log pixel-to-meter ratio
        px_per_meter = rover_width_px / known_rover_width_m
        rospy.loginfo(f"Rover detected: {rover_width_px}px ≈ {known_rover_width_m}m → {px_per_meter:.2f} px/m")



# === VIDEO STREAMING ===
async def capture_video(websocket, camera_manager):
    prev_time = 0
    try:
        while True:
            frame = await camera_manager.read_frame()
            current_time = time.time()
            if current_time - prev_time >= 1 / DESIRED_FPS:
                prev_time = current_time
                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])
                await websocket.send(str(len(buffer)))
                await websocket.send(buffer.tobytes())
            await asyncio.sleep(0.001)
    except Exception as e:
        print(f"Video capture error: {e}")

async def handle_client_commands(websocket, camera_manager):
    try:
        async for switcher in websocket:
            if switcher in ["0", "1", "2", "3", "4"]:
                await camera_manager.switch_feed(int(switcher))
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")

async def stream_video(websocket, camera_manager):
    loop = asyncio.get_running_loop()

    # Start ROS node with access to WebSocket
    def ros_thread():
        JoyPanoramaTrigger(camera_manager, loop, websocket)
        rospy.spin()

    threading.Thread(target=ros_thread, daemon=True).start()

    await asyncio.gather(
        capture_video(websocket, camera_manager),
        handle_client_commands(websocket, camera_manager)
    )


# === MAIN SERVER ===
async def main():
    camera_manager = CameraManager(FEED_INDICES, WIDTH, HEIGHT, DESIRED_FPS)

    server_ip = "192.168.1.23"
    port = 8765
    async with websockets.serve(lambda ws: stream_video(ws, camera_manager), server_ip, port):
        print(f"WebSocket server running at ws://{server_ip}:{port}")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down...")
