#pragma once
#ifdef _WIN32  
#include <windows.h>
#include <chrono>
#include <ctime>
#include <time.h>
#include <stddef.h> 
#include <io.h>  
#elif __linux 
#include <stdio.h>  
#endif 


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace std::chrono;



// 定义Bayer图像中的颜色通道  
enum BayerColor {
    R = 0, // 红色通道  
    Gr,    // 绿色通道（行偶数，列奇数）  
    Gb,    // 绿色通道（行奇数，列偶数）  
    B     // 蓝色通道  
};



//struct _finddata_t
//{
//	unsigned attrib;
//	time_t time_create;
//	time_t time_access;
//	time_t time_write;
//	_fsize_t size;
//	char name[_MAX_FNAME];
//};
std::vector<std::string> vStringSplit(const std::string& s, const std::string& delim);

bool getFilesList(std::string dirs, std::vector<std::vector<std::string>>& files_list);

void getJustCurrentDir(std::string path, std::vector<std::string>& dirs);

void getFiles(std::string path, std::vector<std::string>& files);

std::string GetTimeStamp();

bool SavePointToTxt(cv::Mat deep_map, std::string path, cv::Mat texture_map);

//��ȡz-map Roiͼ
bool MaskZMap(cv::Mat& z_map, cv::Mat mask);

bool MapToColor(cv::Mat deep_map, cv::Mat& color_map, cv::Mat& grey_map, int low_z, int high_z);

bool renderBrightnessImage(cv::Mat brightness, cv::Mat& render_brightness);

bool renderErrorMap(cv::Mat err_map, cv::Mat& color_map, cv::Mat& gray_map, float low_v, float high_v);

bool MergeTextureMap(std::vector<cv::Mat> patterns, cv::Mat& texture_map);

bool compensatePhaseBaseScharr(cv::Mat& normal_phase, cv::Mat brightness, int offset_value);

bool compareNat(const std::string& a, const std::string& b);

bool convertBayer2Gray(cv::Mat bayer, cv::Mat& gray);

bool convertBayer2Blue(cv::Mat bayer, cv::Mat& blue);

bool convertBayer2Rgb(cv::Mat bayer, cv::Mat& rgb);

bool rotate_matrix_to_euler_angles(cv::Mat rotate, cv::Mat &angles);

// Bayer格式转RGB格式  
void bayer2rgb(cv::Mat& bayer, cv::Mat& rgb);

/**
 * @brief BayerBG2RGB，BayerBG格式图像转为RGB图像
 * @param src，输入图像，CV_8UC1格式Bayer图像
 * @param dst，输出图像，CV_8UC3格式彩色图像
 */
void BayerBG2RGB(const cv::Mat& src, cv::Mat& dst);

void bayerBg2Rgb(int width,int height, unsigned char* src, unsigned char* dst);