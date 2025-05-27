import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
ret, frame = cap.read()
if ret:
    cv2.imshow("RealSense RGB", frame)
    cv2.waitKey(0)
else:
    print("Error: Could not capture frame")
cv2.destroyAllWindows()
cv2.waitKey(1)