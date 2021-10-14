import cv2
import numpy as np

def main():
	imgname = "image.png"

	img_height = 480
	img_width = 640
	n_channels = 4
	res = np.zeros((img_height, img_width, n_channels), dtype=np.uint8)

	drawCross(res, 200, 200)

	cv2.imwrite('/home/yashas/Pictures/' + imgname, res)

def drawCross(img, x, y):
     cv2.line(img,(x,y-30),(x,y+30),(0,0,255,255),8)
     cv2.line(img,(x-30,y),(x+30,y),(0,0,255,255),8)

main()
