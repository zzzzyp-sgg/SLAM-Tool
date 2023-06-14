# coding:utf-8
import os
import cv2
from PIL import Image

def img_resize(image):
    height, width = image.shape[0], image.shape[1]
    # 设置新的分辨率
    width_new = 960
    height_new = 600
    img_new = cv2.resize(image, (width_new, height_new))
    return img_new

def main():
    file_path = "/home/zyp/DATA/106_107/107_2/image_00/data"
    file_path_new = "/home/zyp/DATA/106_107/107_2/image_00_dw/data"
    # os.mkdir(file_path_new)
    num_file = len(os.listdir(file_path))
    i=1
    for fileName in os.listdir(file_path):
        image = cv2.imread(file_path + "/" + fileName)
        image_new = img_resize(image)
        image_new = cv2.cvtColor(image_new, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(file_path_new + "/" + fileName, image_new)
        print(str(i) + "/" + str(num_file))
        i = i + 1
    print(image_new.shape)

if __name__ == '__main__':
    main()
