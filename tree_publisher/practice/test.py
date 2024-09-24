import cv2
import numpy as np

class ButtonHandler:
    def __init__(self):
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
                    print(f"Button {i+1} clicked, corresponding to {self.texts[i]}")

def main():
    # テスト用の空白の画像を作成
    img = np.ones((300, 1050, 3), dtype=np.uint8) * 255

    # ボタンのハンドラを作成
    button_handler = ButtonHandler()

    # OpenCVのウィンドウを設定
    cv2.namedWindow("Test Window")
    cv2.setMouseCallback("Test Window", button_handler.on_mouse_click)

    while True:
        img_copy = img.copy()  # 画像のコピーを作成
        button_handler.draw_buttons(img_copy)  # ボタンを描画
        cv2.imshow("Test Window", img_copy)

        # ESCキーで終了
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
