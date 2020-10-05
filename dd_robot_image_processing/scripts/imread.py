import numpy as np
import cv2

# Load an color image in grayscale
#img = cv2.imread('pic.jpg',1)
img = np.array([[[0, 0, 255], [0, 0, 255]], [[0, 255, 0], [0, 255, 0]]])
#cv2.imshow('im',img)
print(img.shape)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
