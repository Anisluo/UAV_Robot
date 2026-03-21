import cv2
import zmq
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://0.0.0.0:5556")

cap = cv2.VideoCapture(4)  # 先试 0，不行换 1、2、4
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("相机打开失败")
    exit()

print("开始推流...")
while True:
    ret, frame = cap.read()
    if not ret:
        continue
    _, buf = cv2.imencode('.jpg', frame)
    socket.send(buf.tobytes())

cap.release()