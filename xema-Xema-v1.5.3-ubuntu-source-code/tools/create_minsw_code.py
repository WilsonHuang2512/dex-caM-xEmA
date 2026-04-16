import csv
import cv2
import numpy as np
import math
import os
import sys

os.chdir(sys.path[0])# 使用文件所在目录


if not os.path.exists('patterns_minsw_code'):
    os.makedirs('patterns_minsw_code')

n = 0 
 
img_x = np.zeros([720, 1280, 3], dtype=np.uint8) 
 
codes = [0, 1, 3, 7, 15, 31, 30, 62, 126, 254, 246, 247, 245, 213, 209, 145,
		153, 152, 136, 8, 40, 42, 43, 35, 99, 103, 71, 70, 68, 76, 204, 220, 252, 253, 189, 185,
		177, 179, 178, 146, 210, 82, 90, 91, 75, 107, 111, 109, 101, 100, 36, 164, 132, 134, 135,
		143, 159, 155, 187, 186, 250, 242, 114, 112, 80, 81, 17, 21, 29, 13, 12, 44, 46, 174, 166,
		167, 231, 199, 195, 193, 201, 200, 216, 88, 120, 56, 57, 49, 51, 55, 23, 22, 86, 94, 222,
		206, 238, 239, 237, 233, 225, 161, 160, 128, 130, 2, 10, 11, 27, 59, 63, 127, 119, 118, 116,
		244, 212, 148, 149, 157, 141, 137, 169, 168, 170, 162, 34, 98, 66, 67, 65, 69, 77, 93, 92,
		124, 60, 188, 180, 181, 183, 151, 147, 211, 219, 218, 202, 74, 106, 104, 105, 97, 33, 37,
		5, 4, 6, 14, 142, 158, 190, 191, 255, 251, 243, 241, 240, 208, 144, 16, 24, 25, 9, 41, 45,
		47, 39, 38, 102, 230, 198, 196, 197, 205, 221, 217, 249, 248, 184, 176, 48, 50, 18, 19, 83,
		87, 95, 79, 78, 110, 108, 236, 228, 229, 165, 133, 129, 131, 139, 138, 154, 26, 58, 122, 123,
		115, 113, 117, 85, 84, 20, 28, 156, 140, 172, 173, 175, 171, 163, 227, 226, 194, 192, 64, 72,
		73, 89, 121, 125, 61, 53, 52, 54, 182, 150, 214, 215, 223, 207, 203, 235, 234, 232, 224, 96, 32 ]

bin_map = np.zeros([8, 256], dtype=np.uint8)

for c_i in range(256):
    b_val = codes[c_i]
    for b_i in range(8):
        switch = int(b_val/pow(2,7-b_i))
        b_val= int(b_val%(pow(2,7-b_i)))
        if 0 == switch:
            bin_map[b_i][c_i] = 0
        else:
            bin_map[b_i][c_i] = 255


# print('patterns_minsw_code\\bin_map_minsw8.bmp')
# cv2.imwrite('patterns_minsw_code\\bin_map_minsw8.bmp',bin_map)
 
minsw_map = np.zeros([720, 1280], dtype=np.uint8)

for p in range(8):  
    for r in range(720): 
        for c in range(256):
            for rep in range(5):
                minsw_map[r,5*c+rep] = bin_map[p][c]
    print('patterns_minsw_code\\minsw_%02d.bmp'%n)
    cv2.imwrite('patterns_minsw_code\\minsw_%02d.bmp'%n, minsw_map)
    n = n+1
n=1
for pixel in [255,  0]:
        img_p = np.zeros([720, 1280], dtype=np.uint8)
        for y_ in range(720):
            for x_ in range(1280):
                img_p[y_, x_] = pixel
        cv2.imwrite('patterns_minsw_code/%02d.bmp' % n, img_p)
        n += 1
        print(n)



