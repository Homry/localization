import cv2
cap = cv2.VideoCapture('http://autolab.moevm.info/camera_1/live.mjpg')

while True:
  ret, frame = cap.read()
  cv2.imshow('Video', frame)
  cv2.imwrite('des.jpg', frame)
  if cv2.waitKey(1) == 27:
    exit(0)