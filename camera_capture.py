# Gets an image from the camera using opencv

from cv2 import *

# initialize the camera
def init_camera():
	cam = VideoCapture(0)   # 0 -> index of camera

def get_image():
	s, img = cam.read()
	if s:    # frame captured without any errors
	    namedWindow("cam-test",WINDOW_AUTOSIZE)
	    imshow("cam-test",img)
	    waitKey(0)
	    destroyWindow("cam-test")
	    imwrite("img.jpg",img) #save image

