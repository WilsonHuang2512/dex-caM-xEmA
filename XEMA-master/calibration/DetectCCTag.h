#pragma once
#include"DetectCCTag.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

bool detectCCTag(cv::Mat img, std::vector<cv::Point2f>& point_list, std::vector<int>& id_list, std::vector<int>& status_list,int i );