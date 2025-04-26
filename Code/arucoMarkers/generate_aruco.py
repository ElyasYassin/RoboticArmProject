import cv2
import sys
import numpy as np
import argparse
#from utils import ARUCO_DICT
import matplotlib.pyplot as plt

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

marker_id = 6
marker_size = 100
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

cv2.imwrite('marker_3.png', marker_image)
plt.imshow(marker_image, cmap='gray', interpolation='nearest')
plt.axis('off')
plt.title(f'ArUco Marker {marker_id}')
plt.show()