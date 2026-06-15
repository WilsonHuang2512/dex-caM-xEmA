#pragma once
#include <opencv2/opencv.hpp>

struct SharpnessResult
{
    bool ok = false;
    cv::Size found_pattern = cv::Size(0, 0); // (cols, rows)
    float lap_abs_mean = 0.0f;               // mean(|lap|)
    float lap_var = 0.0f;                    // var(lap)  (总体方差, N)
    int used_points = 0;                     // 实际参与统计的角点数（<=36）
};

/**
 * @brief 基于棋盘格角点+圆形ROI的投影仪清晰度评价
 * @param img_bgr_or_gray 输入图像(BGR或Gray)
 * @param min_rows 棋盘格最小行
 * @param min_cols 棋盘格最小列
 * @param max_rows 棋盘格最大行
 * @param max_cols 棋盘格最大列
 * @param prev_pattern 上次识别成功的棋盘格(cols, rows)，用于优先尝试；不使用可传(0,0)
 * @param target_points 期望选取靠近中心的角点数（默认36）
 * @param normalize_target_mean 亮度归一化目标平均灰度（0~255，默认128）
 * @param lap_ksize Laplacian核大小（常用1/3/5，默认3）
 * @return SharpnessResult 结果(包含是否成功、识别到的行列、lap_abs_mean、lap_var等)
 */
SharpnessResult computeProjectorSharpnessByChessboard(
    const cv::Mat& img_bgr_or_gray,
    int min_rows, int min_cols,
    int max_rows, int max_cols,
    cv::Size& prev_pattern,
    int target_points = 36,
    double normalize_target_mean = 128.0,
    int lap_ksize = 3
);