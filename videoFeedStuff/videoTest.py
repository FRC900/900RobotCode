import numpy as np
import cv2

cap = cv2.VideoCapture(3)

while(True):
    ret, frame = cap.read()
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grey, 100, 200, 5)
    cv2.imshow("feed", edges)
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        break
cap.release()
cv2.destroyAllWindows()
