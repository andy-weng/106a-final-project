import cv2
import numpy as np

# Create a dummy image
img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.imshow('Test Window', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

