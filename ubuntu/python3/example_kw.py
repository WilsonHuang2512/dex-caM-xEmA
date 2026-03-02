import numpy as np
import cv2
from KW import KW

ip="192.168.9.42"

#保存的参数config.json
config="config.json"
status="status.json"
config_read=[]
status_read=[]
capture_num=[]

camera=KW.Camera()
camera.connect(ip)

#读json文件和写入
ret=camera.readJson(config_read,config)
ret=camera.setParamJson(config_read[0],status_read,capture_num)

# #########################获取通道数和分辨率、分配大小#########################
width = np.zeros(1, dtype=np.int32)
height = np.zeros(1, dtype=np.int32)
channels = np.zeros(1, dtype=np.int32)
ret=camera.getCameraResolution(width,height)
ret=camera.getCameraChannels(channels)
width=int(width[0])
height=int(height[0])
channels=int(channels[0])
bright_data = np.zeros((height, width), dtype=np.uint8)
depth_data = np.zeros((height, width), dtype=np.float32)
color_data = np.zeros((height, width,3), dtype=np.uint8)
pointcloud_data = np.zeros((width*height*3,), dtype=np.float32)

# ####################采集模式的选择##############
camera.captureData(capture_num[0], "time")

# ###########################数据获取模块####################################
if(channels==1):
    ret=camera.getBrightnessData(bright_data)
    ret = camera.getDepthData(depth_data)
    ret = camera.getPointcloudData(pointcloud_data)

    img = np.reshape(bright_data, (height, width))
    depth = np.reshape(depth_data, (height, width))
    cv2.imwrite('bright.bmp', img)
    cv2.imwrite("depth.tiff", depth)

    #ret = camera.savePointcloudToPcd(pointcloud_data, color_data, channels, "1.pcd")
    ret = camera.savePointcloudToPly(pointcloud_data, bright_data, channels, "1.ply")

elif(channels==3):

    ret = camera.getUndistortColorBrightnessData(color_data, KW.Color.Rgb)
    ret = camera.getDepthData(depth_data)
    ret = camera.getPointcloudData(pointcloud_data)

    color_img = np.reshape(color_data, (height, width, 3))
    depth = np.reshape(depth_data, (height, width))

    cv2.imwrite("color.bmp", color_img)
    cv2.imwrite("depth.tiff", depth)


    color_bgr= np.ascontiguousarray(color_img[:, :, ::-1]).reshape(-1)
    #ret = camera.savePointcloudToPcd(pointcloud_data, color_data, channels, "1.pcd")
    ret = camera.savePointcloudToPly(pointcloud_data, color_bgr, channels, "1.ply")


camera.disconnect(ip)



