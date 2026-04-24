import time
import cv2
import numpy as np

def count_cameras(max_tested=10):
    total = 0
    for i in range(max_tested):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]:
            print(f"检测到摄像头，索引编号 ID: {i}")
            total += 1
            cap.release()
        else:
            # 某些情况下 index 可能会跳跃，所以不建议直接 break
            pass
    return total


class Action4():
    def __init__(self,port=2):
        self.dsize = (640, 480)
        self.cap = cv2.VideoCapture(port)
        if self.cap.isOpened():
            # 获取宽度 (属性 ID: 3)
            self.cam_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            # 获取高度 (属性 ID: 4)
            self.cam_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            # 获取帧率 (FPS)
            self.fps = self.cap.get(cv2.CAP_PROP_FPS)
            print(f"摄像头-分辨率: {int(self.cam_width)}x{int(self.cam_height)}，帧率: {self.fps} FPS")
            self.get_shape_information()
    def get_shape_information(self):
        ret, frame = self.cap.read()
        if ret:
            self.height, self.width, self.channels = frame.shape
            print(f"\33[92m画面宽度: {self.width} 像素")
            print(f"画面高度: {self.height} 像素")
            print(f"颜色通道: {self.channels} (通常是 3，代表 BGR 彩色)\33[0m")
    def get_frame(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                return None, None
            return ret, frame
        except Exception as e:
            print(f"获取帧失败: {e}")
            return None, None
    def get_image(self):
        ret, frame = self.get_frame()
        if ret:
            image = np.asanyarray(frame.get_data())
            return image
        else:
            return None
    def display_dsize(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                small_frame = cv2.resize(frame, (640, 480))
                # 显示缩放后的画面
                cv2.imshow('Resized Action 4', small_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    def display(self,windows_name='Action 4'):
        while True:
            ret, frame = self.cap.read()
            if ret:
                # small_frame = cv2.resize(frame, (640, 480))
                #
                # # 显示缩放后的画面
                # cv2.imshow('Resized Action 4 (640x480)', small_frame)

                cv2.imshow(windows_name, frame)
                # height, width, channels = frame.shape
                # print(f"画面宽度: {width} 像素")
                # print(f"画面高度: {height} 像素")
                # print(f"颜色通道: {channels} (通常是 3，代表 BGR 彩色)")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    def end(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    # count_cameras()
    ac4=Action4()
    # ac4.display_dsize()
    while True:
        _, ac4_frame=ac4.get_frame()
        cv2.imshow('Resized Action 4', ac4_frame)
        cv2.waitKey(1)