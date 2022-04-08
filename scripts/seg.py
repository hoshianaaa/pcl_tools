import cv2
import numpy as np

def main():
    image = cv2.imread('image.png')
    image = cv2.blur(image,(3,3))
    print(image.shape)

    # BGR-HSV変換
    converted = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)

    # パラメータ
    region_size = 50
    ruler = 0.05
    min_element_size = 20
    num_iterations = 5

    # LSCインスタンス生成
    slc = cv2.ximgproc.createSuperpixelLSC(converted, region_size,float(ruler))
    slc.iterate(num_iterations)
    slc.enforceLabelConnectivity(min_element_size)

    # スーパーピクセルセグメンテーションの境界を取得
    contour_mask = slc.getLabelContourMask(False)
    image[0 < contour_mask] = (0, 255, 255)
    cv2.imshow('LSC result', image)
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
