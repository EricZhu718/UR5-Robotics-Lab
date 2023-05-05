import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
from time import sleep


if __name__ == '__main__':
    QR_Templates = []
    QR_Pics = []
    for filename in os.listdir('./QR Codes/QR Sample'):
        img_bgr = cv2.imread('./QR Codes/QR Sample/' + filename)

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        QR_Templates.append(img_rgb)
    
    scale_percent = 20
    for filename in os.listdir('./QR Codes/QR Test'):
        
        img_bgr = cv2.imread('./QR Codes/QR Test/' + filename)

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        width = int(img_rgb.shape[1] * scale_percent / 100)
        height = int(img_rgb.shape[0] * scale_percent / 100)
        dim = (width, height)
        # plt.imshow(img_bgr)

        img_rgb = cv2.resize(img_rgb, dim, interpolation = cv2.INTER_AREA)
        QR_Pics.append(img_rgb)

    
    qcd = cv2.QRCodeDetector()
    
    color = (0, 255, 0)
    thickness = 3

    for img in QR_Pics:
        retval, decoded_info, quads_of_points, straight_qrcode = qcd.detectAndDecodeMulti(img)
        # plt.imshow(img)
        # print(quads_of_points)
        quads_of_points = np.int32(quads_of_points)
        # print(quads_of_points)
        for quad in quads_of_points:
            for i in range(len(quad)):
                img = cv2.line(img, quad[i], quad[(i+1) % len(quad)], color, thickness) 
        plt.imshow(img)
        plt.show()
        cv2.waitKey(0)


    
