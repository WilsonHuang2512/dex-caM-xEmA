#pragma once
#ifndef DETECT_CHESSBOARD_H
#define DETECT_CHESSBOARD_H

// 包含必要的头文件
#include <iostream>
#include <opencv2/opencv.hpp>

bool detect_corner(cv::Mat& img, int round_size, std::vector<cv::Point2f>& points);

#endif // DETECT_CHESSBOARD_H