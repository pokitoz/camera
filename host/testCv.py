import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time

## sudo apt install python3-opencv

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')

        self.subscription = self.create_subscription(
            Image, 'img', self.listener_callback, 10)

        self.out = None
        self.frame_rate = 10.0  # frames per second
        self.last_time = time.time()
        self.codec = cv2.VideoWriter_fourcc(*'mp4v')  # or 'XVID' for AVI
        self.output_file = 'output_video.mp4'
        self.frame_size = None  # will set on first frame

    def listener_callback(self, msg: Image):
        # Convert JPEG bytes to a NumPy array
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn("Failed to decode frame.")
            return

        # Setup video writer on first frame
        if self.out is None:
            height, width, _ = frame.shape
            self.frame_size = (width, height)
            self.out = cv2.VideoWriter(
                self.output_file, self.codec, self.frame_rate, self.frame_size)
            self.get_logger().info(f"VideoWriter started: {self.output_file}")

        # Write the frame to video
        self.out.write(frame)
        self.get_logger().info(f"Wrote frame at {time.time() - self.last_time:.2f}s interval")
        self.last_time = time.time()

    def destroy_node(self):
        # Release video writer
        if self.out:
            self.out.release()
            self.get_logger().info("Video saved and writer released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
