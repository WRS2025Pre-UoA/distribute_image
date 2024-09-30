import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
# 分配器でキー入力で画像を送る


class ButtonHandler:
    def __init__(self):
        # ボタンの位置とサイズ
        self.button_positions = []
        baseX = 10
        diffX = 120
        baseY = 45
        diffY = 70

        for y in range(3):
            for x in range(2):
                pos_x = baseX+diffX*x
                pos_y = baseY+diffY*y
                self.button_positions.append((pos_x, pos_y))
        self.button_width = 110
        self.button_height = 50
        # 各ボタンに対応するテキスト
        self.texts = ["Meter", "QR", "Rust", "Crack", "Temp", "Send"]

    def draw_buttons(self, img):
        # 背景に黒の長方形を描画 (250x280)
        # background_top_left = (0, 0)
        # background_bottom_right = (250, 280)
        # cv2.rectangle(img, background_top_left, background_bottom_right, (0, 0, 0), -1)

        # 各ボタンを描画
        for i, pos in enumerate(self.button_positions):
            top_left = pos
            bottom_right = (top_left[0] + self.button_width,
                            top_left[1] + self.button_height)
            # 白でボタンを描画
            cv2.rectangle(img, top_left, bottom_right, (255, 255, 255), -1)
            # ボタン上にテキストを描画
            cv2.putText(img, self.texts[i], (top_left[0] + 10, top_left[1] + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def clicked_image(self, x, y, send_hundlers, values):
        for i, pos in enumerate(self.button_positions):
            top_left = pos
            bottom_right = (top_left[0] + self.button_width,
                            top_left[1] + self.button_height)
            if top_left[0] <= x <= bottom_right[0] and top_left[1] <= y <= bottom_right[1]:
                print(
                    f"Button {i+1} clicked, sending image to {self.texts[i]}")
                send_hundlers[i].publish(values[i])


class DistributeImage(Node):
    def __init__(self):
        super().__init__('distribute_image')
        self.image_publishers = [
            self.create_publisher(Image, 'pressure_image', 1),
            self.create_publisher(Image, 'qr_image', 1),
            self.create_publisher(Image, 'rust_image', 1),
            self.create_publisher(Image, 'crack_image', 1),
            self.create_publisher(Image, 'temperature_image', 1),
            self.create_publisher(String, 'send_topic', 1),
        ]
        self.bridge = CvBridge()

        # GUI
        self.button_handler = ButtonHandler()

        self.timer = self.create_timer(0.1, self.publish_gui)
        self.gui_publisher = self.create_publisher(Image, 'gui', 1)
        self.mouse_click_subscription = self.create_subscription(
            Point,
            'gui_mouse_left',
            self.mouse_click_callback,
            10
        )

        # 画像受け取り
        self.last_received_image = None
        self.cv_image = None  # OpenCV形式の画像（表示用）

        self.image_subscription = self.create_subscription(
            Image,
            'input_image_topic',
            self.image_callback,
            10
        )

        # 結果受け取り(表示用)
        self.result_status = "None"
        self.result_value = ""
        self.result_qr = None

        self.current_result_status_subscription = self.create_subscription(
            String,
            'current_result_status',
            self.current_result_status_callback,
            10
        )
        self.current_result_value_subscription = self.create_subscription(
            Float64,
            'current_result_value',
            self.current_result_value_callback,
            10
        )
        self.current_result_qr_subscription = self.create_subscription(
            String,
            'current_result_qr',
            self.current_result_qr_callback,
            10
        )

    def publish_gui(self):
        if self.cv_image is not None:
            # 黒い長方形の背景画像を作成
            img_with_buttons = np.zeros((350, 250, 3), dtype=np.uint8)
            # テキスト「MISORA 2」を高さ70以内に中央に描画
            text = "MISORA 2"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.5
            font_thickness = 2
            text_size = cv2.getTextSize(
                text, font, font_scale, font_thickness)[0]
            text_x = (img_with_buttons.shape[1] - text_size[0]) // 2  # 中央に配置
            text_y = 35  # 高さ70以内の中央
            # 黒い背景に白で文字を描画
            cv2.putText(img_with_buttons, text, (text_x, text_y),
                        font, font_scale, (255, 255, 255), font_thickness)
            self.button_handler.draw_buttons(img_with_buttons)

            # 最新の結果の表示
            result_text_x = 20
            result_text_y = 270
            cv2.putText(img_with_buttons, f"{self.result_status}", (result_text_x, result_text_y),
                        font, 1, (255, 255, 255), 1)
            cv2.putText(img_with_buttons, f"QR: {self.result_qr}", (result_text_x+10, result_text_y+25),
                        font, 0.7, (255, 255, 255), 1)
            result_value_text = f"Result: {self.result_value}"
            if self.result_status == "None":
                result_value_text = f"Result: None"
            cv2.putText(img_with_buttons, result_value_text, (result_text_x+10, result_text_y+50),
                        font, 0.7, (255, 255, 255), 1)

            try:
                selection_image = self.bridge.cv2_to_imgmsg(
                    img_with_buttons, 'bgr8')
                self.gui_publisher.publish(selection_image)
            except CvBridgeError as e:
                print(e)

    def mouse_click_callback(self, point):
        self.get_logger().info("Point received")
        if self.last_received_image != None:
            values = (len(self.image_publishers)-1) * \
                [self.last_received_image]+[String()]
            self.button_handler.clicked_image(
                point.x, point.y, self.image_publishers, values)

    def image_callback(self, msg):
        self.last_received_image = msg
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(e)

    def current_result_status_callback(self, msg):
        self.result_status = msg.data

    def current_result_value_callback(self, msg):
        self.result_value = msg.data

    def current_result_qr_callback(self, msg):
        self.result_qr = msg.data


def main(args=None):
    rclpy.init(args=args)

    node = DistributeImage()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
