import cv2
from cv_bridge import CvBridge
import numpy as np
import sys

def detectFeature(filename):
    image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)

    max_corners = 240

    corners = cv2.goodFeaturesToTrack(image, max_corners, 0.01, 20)

    if corners is not None:
        corners = np.intp(corners)

        for corner in corners:
            x, y = corner.ravel()
            cv2.circle(image, (x, y), 3, (0, 0, 255), -1)

        cv2.imshow("Corners", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return len(corners)
    else:
        print("No corners were detected")


if __name__ == '__main__':
    if(len(sys.argv) == 1):
        print("Please select a file!")
        exit(1)
    filename = sys.argv[1]

    featureNum = detectFeature(filename)
    print(featureNum)
