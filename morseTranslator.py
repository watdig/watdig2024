import cv2

cap = cv2.VideoCapture('WIN_20240217_21_15_57_Pro.mp4')

while True:

    success, img = cap.read()
    cv2.imshow("Image", img)
    cv2.waitKey(1)
