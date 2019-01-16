import numpy as np
import cv2
from imutils.video import FPS
from imutils.video import WebcamVideoStream
import datetime

# From https://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

#cap = cv2.VideoCapture(0)
cap = WebcamVideoStream(src=1).start()
fps = FPS().start()
# Default resolutions of the frame are obtained.The default resolutions are system dependent.
# We convert the resolutions from float to integer.
count = 0
while(True):
    frame = cap.read()
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.GaussianBlur(grey, (3, 3), 0)

    # Bilateral filter - tends to clean up carpet-related
    # noise from the edge detection code
    blurred = cv2.bilateralFilter(grey,9,75,75)
    # auto seems to be a bit better, but keep both in 
    # place in case there's a need to compare
    edges = cv2.Canny(blurred, 100, 200, 5)
    auto = auto_canny(blurred)
    cv2.imshow("feed", edges)
    cv2.imshow("auto", auto)
    cv2.imshow("blurred", blurred)

    # pyrDown is a quick way to cut resolution in half
    # in both directions.  A low-res greyscale image is
    # another option for streaming video, especially if
    # we resize it back up on the DS - it'll be blurry
    # but usable?  Depends on bandwidth
    down = cv2.pyrDown(blurred)
    edges_down = cv2.Canny(down, 100, 200, 5)
    auto_down = auto_canny(down)
    cv2.imshow("feed_down", edges_down)
    cv2.imshow("auto_down", auto_down)
    cv2.imshow("down down down down", down)

    # Here's what a blurry upsampled image would look 
    # like, for testing / evaluation purposes.
    up = cv2.pyrUp(down)
    cv2.imshow("up", up)
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
