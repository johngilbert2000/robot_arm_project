import cv2
import sys
cam = cv2.VideoCapture(0)
ret, frame = cam.read()
cv2.imwrite(sys.argv[1], frame)
cam.release()
cv2.destroyAllWindows()