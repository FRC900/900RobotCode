import numpy as np
import cv2
from imutils.video import FPS
from imutils.video import WebcamVideoStream
import datetime


#cap = cv2.VideoCapture(0)
cap = WebcamVideoStream(src=0).start()
fps = FPS().start()

while(True):
    frame = cap.read()
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.bilateralFilter(grey, 5, 200, 200)
    edges = cv2.Canny(blur, 100, 200, 5)
    #cv2.imshow("grey", grey)
    #cv2.imshow("blur", blur)
    cv2.imshow("edges", edges)
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        	break
    fps.update()
fps.stop()
print("elapsed time")
print(fps.elapsed())
print("average FPS")
print(fps.fps())
cap.stop()
cv2.destroyAllWindows()
