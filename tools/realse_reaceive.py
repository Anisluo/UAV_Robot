import cv2
import zmq
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.10.2:5556")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

print("连接中，等待画面...")
while True:
    buf = socket.recv()
    arr = np.frombuffer(buf, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        continue
    cv2.imshow("RealSense D435", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()