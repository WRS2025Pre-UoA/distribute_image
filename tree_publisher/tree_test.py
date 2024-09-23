import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os
import time
import cv2
import numpy as np

class TestImagePublisher(Node):
    def __init__(self, directory_path):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, 'input_image_topic', 10)
        self.directory_path = directory_path
        self.image_files = [f for f in os.listdir(directory_path) if f.endswith('.jpg')]
        self.index = 0
        self.get_logger().info("Test Image Publisher Node Initialized")

    def publish_image(self):
        if self.index >= len(self.image_files):
            self.get_logger().info("All images sent.")
            return

        image_path = os.path.join(self.directory_path, self.image_files[self.index])
        self.get_logger().info(f"Sending image: {image_path}")

        # 画像をOpenCVで読み込む
        img = cv2.imread(image_path)
        if img is None:
            self.get_logger().warn(f"Failed to load image: {image_path}")
            return

        # 画像をROS2のImageメッセージに変換
        ros_image = self.convert_cv2_to_ros_image(img)

        # トピックに画像を送信
        self.publisher.publish(ros_image)

        # 次の画像にインデックスを進める
        self.index += 1

    def convert_cv2_to_ros_image(self, cv_image):
        # OpenCV画像からROS2のImageメッセージに変換
        ros_image = Image()
        ros_image.height, ros_image.width, channels = cv_image.shape
        ros_image.encoding = "bgr8"  # OpenCVの標準フォーマット（BGR）
        ros_image.data = cv_image.tobytes()
        ros_image.step = channels * ros_image.width
        return ros_image

def main(args=None):
    rclpy.init(args=args)

    # ディレクトリパスの設定
    directory_path = "/home/ros/ros2_ws/src/pressure/data/0908_results"  # ここに.jpg画像があるディレクトリパスを設定

    image_publisher = TestImagePublisher(directory_path)

    try:
        while rclpy.ok() and image_publisher.index < len(image_publisher.image_files):
            image_publisher.publish_image()
            time.sleep(10)  # 10秒毎に画像を送信
    except KeyboardInterrupt:
        pass

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
