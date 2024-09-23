import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
# 分配器でキー入力で画像を送る
class ImageReceiver(Node):
    def __init__(self):
        super().__init__('image_receiver')
        self.subscription = self.create_subscription(
            Image,
            'input_image_topic',
            self.image_callback,
            10
        )
        self.subscription = self.create_subscription(
            Point,
            'gui_mouse_click',
            self.mouse_click_callback,
            10
        )
        self.image = None
        self.cv_image = None  # OpenCV形式の画像（表示用）
        self.get_logger().info("Image Receiver Node Initialized")

        self.point=None

    def image_callback(self, msg):
        self.image = msg
        # ROS2のImageメッセージをOpenCV形式に変換
        self.cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.get_logger().info("Image received")
    
    def mouse_click_callback(self, msg):
        self.point = msg
        # ROS2のImageメッセージをOpenCV形式に変換
        self.get_logger().info("Point received")

    def get_cv_image(self):
        return self.cv_image

    def get_ros_image(self):
        return self.image

class ButtonHandler:
    def __init__(self, image_sender):
        self.image_sender = image_sender
        self.button_positions = [(50, 50), (250, 50), (450, 50), (650, 50), (850, 50)]
        self.button_width = 150
        self.button_height = 100
        self.texts = ["Topic1", "Topic2", "Topic3", "Topic4", "Topic5"]

    def draw_buttons(self, img):
        for i, pos in enumerate(self.button_positions):
            top_left = pos
            bottom_right = (top_left[0] + self.button_width, top_left[1] + self.button_height)
            cv2.rectangle(img, top_left, bottom_right, (0, 0, 255), 2)
            cv2.putText(img, self.texts[i], (top_left[0] + 10, top_left[1] + 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for i, pos in enumerate(self.button_positions):
                top_left = pos
                bottom_right = (top_left[0] + self.button_width, top_left[1] + self.button_height)
                if top_left[0] <= x <= bottom_right[0] and top_left[1] <= y <= bottom_right[1]:
                    print(f"Button {i+1} clicked, sending image to {self.texts[i]}")
                    # ボタンに対応するトピックに画像を送信
                    self.image_sender.send_image_to_topic(i + 1)
    
    def clicked_image(self, x, y):
        for i, pos in enumerate(self.button_positions):
            top_left = pos
            bottom_right = (top_left[0] + self.button_width, top_left[1] + self.button_height)
            if top_left[0] <= x <= bottom_right[0] and top_left[1] <= y <= bottom_right[1]:
                print(f"Button {i+1} clicked, sending image to {self.texts[i]}")
                # ボタンに対応するトピックに画像を送信
                self.image_sender.send_image_to_topic(i + 1)

class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')
        self.publisher1 = self.create_publisher(Image, 'topic1', 10)
        self.publisher2 = self.create_publisher(Image, 'topic2', 10)
        self.publisher3 = self.create_publisher(Image, 'topic3', 10)
        self.publisher4 = self.create_publisher(Image, 'topic4', 10)
        self.publisher5 = self.create_publisher(Image, 'topic5', 10)
        self.gui_publish = self.create_publisher(Image, 'gui', 10)
        self.last_received_image = None

    def update_last_received_image(self, image):
        self.last_received_image = image

    def send_image_to_topic(self, topic_number):
        if self.last_received_image is not None:
            if topic_number == 1:
                self.publisher1.publish(self.last_received_image)
            elif topic_number == 2:
                self.publisher2.publish(self.last_received_image)
            elif topic_number == 3:
                self.publisher3.publish(self.last_received_image)
            elif topic_number == 4:
                self.publisher4.publish(self.last_received_image)
            elif topic_number == 5:
                self.publisher5.publish(self.last_received_image)
            self.get_logger().info(f"Image sent to topic{topic_number}")
        else:
            self.get_logger().info("No image received to send.")

def main(args=None):
    rclpy.init(args=args)
    
    image_receiver = ImageReceiver()
    image_sender = ImageSender()
    button_handler = ButtonHandler(image_sender)

    bridge = CvBridge()

    # cv2.namedWindow("Image with Buttons")
    # cv2.setMouseCallback("Image with Buttons", button_handler.on_mouse_click)

    while rclpy.ok():
        rclpy.spin_once(image_receiver,timeout_sec=1)

        cv_image = image_receiver.get_cv_image()
        ros_image = image_receiver.get_ros_image()

        if cv_image is not None:
            # 画像のコピーを作成してボタンを描画
            img_with_buttons = cv_image.copy()
            button_handler.draw_buttons(img_with_buttons)

            # OpenCVで画像を表示
            # cv2.imshow("Image with Buttons", img_with_buttons)
            # cv2.waitKey(1)  # OpenCVの画面を更新
            try:
                selection_image = bridge.cv2_to_imgmsg(img_with_buttons, 'bgr8')
                image_sender.gui_publish.publish(selection_image)
            except CvBridgeError as e:
                print(e)
            
            if image_receiver.point != None:
                button_handler.clicked_image(image_receiver.point.x, image_receiver.point.y)
                image_receiver.point=None

            # 最新の画像をImageSenderに渡す
            image_sender.update_last_received_image(ros_image)

    image_receiver.destroy_node()
    image_sender.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
