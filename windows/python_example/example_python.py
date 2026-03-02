import sdkpython
import numpy as np
import cv2

#保存xyz点云无颜色
def save_xyz_file(filename, data, width, height):
    with open(filename, 'w') as file:
        for i in range(width*height):
            file.write(str(data[i * 3]) + " " + str(data[i * 3 + 1]) + " " + str(data[i * 3 + 2]) + "\n")


#保存xyz点云带物体颜色
def save_color_xyz_file(filename, data,bright, width, height):
    bright_data=[]
    for i in range(1200):
         for j in range(1920):
             bright_data.append(bright[i][j])
    with open(filename, 'w') as file:
        for s in range(width*height):
            file.write(str(data[s * 3]) + " " + str(data[s * 3 + 1]) + " " + str(data[s * 3 + 2]) + " " + str(bright_data[s]) + " " + str(bright_data[s]) + " " + str(bright_data[s]) + "\n")


#保存ply点云带物体颜色
def save_color_ply_file(input_cloud_file):

    # 1. 读取数据
    data = []
    with open(input_cloud_file, 'r') as f:
        lines = f.readlines()
        for line in lines:
            values = line.split()
            x, y, z, r, g, b = map(float, values)
            data.append([x, y, z, r, g, b])
    # 2. 转换为NumPy数组
    data = np.array(data)
    # 3. 创建文件头部信息
    header = 'ply\n'
    header += 'format ascii 1.0\n'
    header += 'element vertex %d\n' % len(data)
    header += 'property float x\n'
    header += 'property float y\n'
    header += 'property float z\n'
    header += 'property uchar red\n'
    header += 'property uchar green\n'
    header += 'property uchar blue\n'
    header += 'end_header\n'
    # 4. 写入数据
    with open('output.ply', 'w') as f:
        f.write(header)
        for row in data:
            f.write('%f %f %f %d %d %d\n' % (row[0], row[1], row[2], row[3], row[4], row[5]))
    # 5. 关闭文件并保存
    f.close()

#调用n次
for i in range(1):
    print(f'第{i}次循环')
    num=0
    print('mac地址及ip如下：')
    sdkpython.DfUpdateDeviceList(num)

    #连接相机（根据mac地址自己选择IP）
    ip='192.168.3.54'
    ret=sdkpython.DfConnect(ip)

    #确定相机已经连接
    if(ret==0):
        #height=1920
        #width=1200
        print('连接相机成功！')

        #相机参数结构体
        calib_param = sdkpython.CalibrationParam()
        #获取相机参数
        sdkpython.DfGetCalibrationParam(calib_param)
        #获取相机内参
        intrinsic = sdkpython.get_intrinsic(calib_param)
        print('相机内参：',intrinsic)
        # 获取相机外参
        extrinsic = sdkpython.get_extrinsic(calib_param)
        print('相机外参：', extrinsic)
        # 获取相机畸变
        distortion = sdkpython.get_distortion(calib_param)
        print('相机畸变：', distortion)

        width=np.zeros(1,dtype=np.int32)
        height=np.zeros(1,dtype=np.int32)
        cannels=np.zeros(1,dtype=np.int32)
        sdkpython.DfGetCameraResolution(width,height)
        sdkpython.DfGetCameraChannels(cannels)
        print('通道数',cannels[0])
        print("height:",height[0])
        print("width:", width[0])

        width=int(width[0])
        height=int(height[0])

        #设置曝光时间
        sdkpython.DfSetParamCameraExposure(10000)
        #设置投影亮度
        sdkpython.DfSetParamLedCurrent(1023)

        #设置彩色参数
        sdkpython.DfSetParamGenerateBrightness(1,20000)

        ##设置相关参数
        #置信度
        sdkpython.DfSetParamCameraConfidence(5)
        #增益
        sdkpython.DfSetParamCameraGain(2)
        #平滑
        sdkpython.DfSetParamSmoothing(5)
        #引擎
        sdkpython.DfSetCaptureEngine(sdkpython.XemaEngine.Reflect)
        #深度滤波
        sdkpython.DfSetParamDepthFilter(1,33)
        #相位校正
        sdkpython.DfSetParamGrayRectify(1,3,33)
        #噪点过滤
        sdkpython.DfSetParamOutlierFilter(30)
        #半径滤波
        sdkpython.DfSetParamRadiusFilter(1,6,20)
        #反射滤波
        sdkpython.DfSetParamReflectFilter(1,50)

        #采集一帧数据并阻塞（单曝光）
        #sdkpython.DfCaptureData(1,'2023-7-20')

        # 采集一帧数据并阻塞（重复曝光）
        '''sdkpython.DfSetParamMultipleExposureModel(2)
        sdkpython.DfSetParamRepetitionExposureNum(2)
        sdkpython.DfCaptureData(2, '2023.7.15')'''

        #采集一帧数据并阻塞（高动态HDR）
        exposure_num=[2000,30000,60000,60000,60000,60000]
        led_num=[1023,1023,1023,1023,1023,1023]
        sdkpython.DfSetParamMixedHdr(2,exposure_num,led_num)
        #sdkpython.DfSetParamMultipleExposureModel(1)
        sdkpython.DfCaptureData(2,'hh')




        #数组创建
        # 创建一个大小为 (height, width) 的全零的 float 类型数组
        bright_data = np.zeros((width, height), dtype=np.uint8)
        depth_data=np.zeros((width,height),dtype=np.uint16)
        color_data=np.zeros((width,height,cannels[0]),dtype=np.uint8)

        # 获取亮度图
        sdkpython.DfGetUndistortBrightnessData(bright_data)
        # 展示并保存
        '''bright = cv2.cvtColor(bright_data, cv2.COLOR_GRAY2BGR)
        cv2.namedWindow("bmp", cv2.WINDOW_NORMAL)  # 创建一个可调整大小的窗口
        cv2.resizeWindow("bmp", 800, 600)  # 设置窗口大小为800x600像素
        cv2.imshow("bmp", bright)  # 显示图像
        cv2.waitKey(1000)
        cv2.destroyAllWindows()'''
        #保存 路径自己修改
        #cv2.imwrite(f'D:\\haihang\\C++ code\\pybind11\\python\\capture\\{i}.bmp', bright)

        # 获取彩色图
        '''print('开始获取彩色')
        sdkpython.DfGetColorBrightnessData(color_data, sdkpython.XemaColor.Bgr)
        sdkpython.DfGetUndistortColorBrightnessData(color_data,sdkpython.XemaColor.Bgr)'''
        # 展示并保存
        '''color = cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR)
        cv2.imshow("color", color)
        cv2.waitKey(0)'''

        #获取深度图
        sdkpython.DfGetDepthData(depth_data)
        #展示并保存
        '''depth=cv2.cvtColor(depth_data,cv2.COLOR_GRAY2BGR)
        cv2.namedWindow("depth", cv2.WINDOW_NORMAL)  # 创建一个可调整大小的窗口
        cv2.resizeWindow("depth", 800, 600)  # 设置窗口大小为800x600像素
        cv2.imshow("depth", depth)  # 显示图像
        cv2.waitKey(1000)
        cv2.destroyAllWindows()'''
        #保存 路径自己修改
        #cv2.imwrite(f'D:\\haihang\\C++ code\\pybind11\\python\\capture\\{i}.tiff',depth)

        #点云采集
        pointcloud_data = np.zeros((width*height*3,), dtype=np.float32)
        ret=sdkpython.DfGetPointcloudData(pointcloud_data)
        #路径自己修改
        '''filename=f"D:\\haihang\\C++ code\\pybind11\\python\\capture\\{i}.xyz"
        save_xyz_file(filename, pointcloud_data, width, height)
        save_color_xyz_file(filename, pointcloud_data,bright_data ,width, height)
        save_color_ply_file(filename)'''


        #断开相机

        finish=sdkpython.DfDisconnect(ip)
        if(finish==0):
            print('相机已断开')




