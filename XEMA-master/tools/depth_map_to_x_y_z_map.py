import numpy as np
import cv2
import sys

def depthmap_status(img):
    mask = (img > 0.001)
    valid_img = img[mask]
    valid_img = valid_img.reshape(-1)
    depth_min = np.percentile(valid_img, 2)
    depth_max = np.percentile(valid_img, 98)
    return depth_max, depth_min, mask

def Loading_Depth_From_Tiff(depth_file):
    depth = cv2.imread(depth_file, -1)
    depth = np.float32(np.array(depth))

    return depth


def Loading_Params_From_Txt(params_file):
    params = np.loadtxt(params_file)
    camera_mtx = np.zeros((3, 3))
    camera_mtx[0, 0] = params[0]
    camera_mtx[0, 2] = params[2]
    camera_mtx[1, 1] = params[4]
    camera_mtx[1, 2] = params[5]
    camera_mtx[2, 2] = 1
    camera_dist = params[9:14]

    return camera_mtx, camera_dist

def depth2xyz_undistort(depth, camera_mtx, camera_dist):
    x_map = np.zeros(depth.shape, dtype=np.float32)
    y_map = np.zeros(depth.shape, dtype=np.float32)
    z_map = np.zeros(depth.shape, dtype=np.float32)

    for iy in range(depth.shape[0]):
        for ix in range(depth.shape[1]):
            # 先对图片去畸变
            ixiy = cv2.undistortPoints(np.float32(np.array([ix, iy])), camera_mtx, camera_dist)
            z = depth[iy, ix]
            if z > 0:
                x = ixiy[0, 0, 0] * z
                y = ixiy[0, 0, 1] * z
                x_map[iy, ix] = x
                y_map[iy, ix] = y
                z_map[iy, ix] = z

    return x_map, y_map, z_map

def depth_to_x_y_z(depth_tiff_path, param_txt_path):
    camera_mtx, camera_dist = Loading_Params_From_Txt(param_txt_path)
    depth = Loading_Depth_From_Tiff(depth_tiff_path)

    map_x, map_y, map_z = depth2xyz_undistort(depth, camera_mtx, camera_dist)

    output_x_path = depth_tiff_path[:-5]+'_x_map.tiff'
    output_y_path = depth_tiff_path[:-5]+'_y_map.tiff'
    output_z_path = depth_tiff_path[:-5]+'_z_map.tiff'

    print(output_x_path)
    print(output_y_path)
    print(output_z_path)

    cv2.imwrite(output_x_path, map_x)
    cv2.imwrite(output_y_path, map_y)
    cv2.imwrite(output_z_path, map_z)

if __name__ == '__main__':
    argv_lenth = len(sys.argv)
    if argv_lenth == 1:
        print("使用示例：python depth_map_to_x_y_z_map.py ./depth.tiff param.txt")
        exit(0)

    input_tiff_path = sys.argv[1]
    input_param_path = sys.argv[2]

    print(input_tiff_path)

    depth_to_x_y_z(input_tiff_path, input_param_path)
