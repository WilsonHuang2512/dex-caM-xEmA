#include "projector_sharpness.h"
#include "detect_chessboard.h"

#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

/* ------------------- helper: 选出最靠近中心的K个点 ------------------- */
static std::vector<cv::Point2f> pickNearestToCenter(
    const std::vector<cv::Point2f>& pts,
    const cv::Point2f& center,
    int K)
{
    if ((int)pts.size() <= K) return pts;

    std::vector<int> idx(pts.size());
    std::iota(idx.begin(), idx.end(), 0);

    std::nth_element(idx.begin(), idx.begin() + K, idx.end(),
        [&](int a, int b) {
            double da = cv::norm(pts[a] - center);
            double db = cv::norm(pts[b] - center);
            return da < db;
        });

    idx.resize(K);
    std::vector<cv::Point2f> out;
    out.reserve(K);
    for (int i : idx) out.push_back(pts[i]);
    return out;
}

/* ------------------- helper: 计算角点两两最小距离 ------------------- */
static double minPairwiseDistance(const std::vector<cv::Point2f>& pts)
{
    if (pts.size() < 2) return 0.0;

    double dmin = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < pts.size(); ++i)
    {
        for (size_t j = i + 1; j < pts.size(); ++j)
        {
            double d = cv::norm(pts[i] - pts[j]);
            if (d < dmin) dmin = d;
        }
    }
    return std::isfinite(dmin) ? dmin : 0.0;
}

/* ------------------- helper: 创建单个圆形 ROI mask ------------------- */
static cv::Mat makeCircleMask(const cv::Size& size, const cv::Point2f& c, int radius)
{
    cv::Mat mask(size, CV_8U, cv::Scalar(0));
    cv::circle(mask, c, radius, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
    return mask;
}

/* ===================== main ===================== */
SharpnessResult computeProjectorSharpnessByChessboard(
    const cv::Mat& img_bgr_or_gray,
    int min_rows, int min_cols,
    int max_rows, int max_cols,
    cv::Size& prev_pattern,
    int target_points,
    double normalize_target_mean,
    int lap_ksize)
{
    SharpnessResult res;
    if (img_bgr_or_gray.empty()) return res;

    // 1) 转灰度
    cv::Mat gray;
    if (img_bgr_or_gray.channels() == 1) {
        gray = img_bgr_or_gray;
    }
    else {
        cv::cvtColor(img_bgr_or_gray, gray, cv::COLOR_BGR2GRAY);
    }

    // 2) 找棋盘格
    std::cout << "start find chessboard" << std::endl;
    cv::Size found_pattern;
    std::vector<cv::Point2f> corners;
    if (!detect_corner(gray, target_points, corners))
    {
        return res; // ok=false
    }

    std::cout << "success this time" << std::endl;
    prev_pattern = found_pattern;

    res.found_pattern = found_pattern;

    // 选取靠近中心的 target_points 个角点
    cv::Point2f center((float)(gray.cols * 0.5), (float)(gray.rows * 0.5));
    std::vector<cv::Point2f> sel = pickNearestToCenter(corners, center, target_points);
    res.used_points = (int)sel.size();
    // 3) ROI 半径：r = 0.9 * (dmin/2)
    double dmin = minPairwiseDistance(sel);
    if (dmin <= 1e-6) return res;

    int radius = (int)std::floor(0.9 * 0.5 * dmin);
    radius = std::max(radius, 2);

    // 生成 ROI masks
    std::vector<cv::Mat> roi_masks;
    roi_masks.reserve(sel.size());
    std::vector<int> roi_area;
    roi_area.reserve(sel.size());

    for (const auto& p : sel)
    {
        cv::Mat m = makeCircleMask(gray.size(), p, radius);
        roi_masks.push_back(m);
        roi_area.push_back(cv::countNonZero(m));
    }

    // 4) 亮度归一化（ROI 独立统计，允许重叠重复计数）
    double sum_intensity = 0.0;
    double sum_pixels = 0.0;

    for (size_t i = 0; i < roi_masks.size(); ++i)
    {
        const cv::Mat& m = roi_masks[i];
        cv::Scalar mu = cv::mean(gray, m);                 // mask 内均值
        sum_intensity += mu[0] * (double)roi_area[i];      // 均值 * 面积 = 和
        sum_pixels += (double)roi_area[i];
    }

    if (sum_pixels <= 1.0) return res;

    const double mean_roi = sum_intensity / sum_pixels;
    const double ratio = (mean_roi > 1e-6) ? (normalize_target_mean / mean_roi) : 1.0;

    cv::Mat gray_norm_f;
    gray.convertTo(gray_norm_f, CV_8U, ratio, 0.0);

    // 5) Laplacian
    cv::Mat lap;
    cv::Laplacian(gray_norm_f, lap, CV_64F, lap_ksize);

    // 6) 基于 ROI 统计 Laplacian（允许重复采样）
    //    这里不必真的存 huge vector，也可以在线累计一阶/二阶矩；
    //    但你明确说要放入 vector，我这里仍按 vector 的语义做统计（同时也在线统计，避免两遍遍历）。
    double sum_abs = 0.0;   // sum(|lap|)
    double sum_v = 0.0;   // sum(lap)
    double sum_v2 = 0.0;   // sum(lap^2)
    long long N = 0;

    for (const auto& m : roi_masks)
    {
        for (int y = 0; y < lap.rows; ++y)
        {
            const uchar* mp = m.ptr<uchar>(y);
            const double* lp = lap.ptr<double>(y);
            for (int x = 0; x < lap.cols; ++x)
            {
                if (!mp[x]) continue;
                const double v = lp[x];
                sum_abs += std::abs(v);
                sum_v += v;
                sum_v2 += v * v;
                ++N;
            }
        }
    }

    if (N <= 0) return res;

    const double mean_abs = sum_abs / (double)N;          // mean(|lap|)
    const double mean_v = sum_v / (double)N;            // mean(lap)
    const double var_v = (sum_v2 / (double)N) - (mean_v * mean_v); // var(lap)

    res.lap_abs_mean = (float)mean_abs;
    res.lap_var = (float)var_v;
    res.ok = true;
    return res;
}