#include<opencv2/opencv.hpp>
#include<iostream>
#include "detect_chessboard.h"

cv::Point2f centerContour(std::vector<cv::Point>& contours)
{
	cv::Rect rect = cv::boundingRect(contours);
	return cv::Point2f(rect.x + rect.width / 2.0f,
		rect.y + rect.height / 2.0f);
}

void find_nereast_rect(std::vector<std::vector<cv::Point>>& contourIn,
	std::vector<std::vector<cv::Point>>& contourOut, int n, cv::Point2f& center)
{
	std::vector<float>min_dists;
	min_dists.assign(n, FLT_MAX);
	for (size_t i = 0; i < contourIn.size(); i++) {

		auto ct = centerContour(contourIn[i]);
		for (int k = 0; k < n; ++k)
		{
			if (cv::norm(ct - center) < min_dists[k] && cv::norm(ct - center) > 3)
			{
				for (int m = n - 1; m > k; --m)
				{
					contourOut[m] = std::move(contourOut[m - 1]);
					min_dists[m] = min_dists[m - 1];
				}
				contourOut[k] = contourIn[i];
				min_dists[k] = cv::norm(ct - center);
				break;
			}
		}
	}
}

bool approx_bounding(std::vector<cv::Point>& contour, std::vector<cv::Point>& approx_contour)
{
	const int min_approx_level = 1, max_approx_level = 7;
	for (int approx_level = min_approx_level; approx_level <= max_approx_level; approx_level++)
	{
		approxPolyDP(contour, approx_contour, (float)approx_level, true);
		if (approx_contour.size() == 4)
			return true;

		// we call this again on its own output, because sometimes
		// approxPoly() does not simplify as much as it should.
		std::vector<cv::Point> approx_contour_tmp;
		std::swap(approx_contour, approx_contour_tmp);
		approxPolyDP(approx_contour_tmp, approx_contour, (float)approx_level, true);
		if (approx_contour.size() == 4)
			return true;
	}
	return false;
}

double pointToLineDistance(const cv::Point2f& point,
	const cv::Point2f& linePoint1,
	const cv::Point2f& linePoint2)
{
	// 计算向量
	cv::Point2f lineVec = linePoint2 - linePoint1;
	cv::Point2f pointVec = point - linePoint1;

	// 叉积的模 = |lineVec| * |pointVec| * sin(theta)
	double crossProduct = std::abs(lineVec.x * pointVec.y - lineVec.y * pointVec.x);

	// 直线长度
	double lineLength = cv::norm(lineVec);

	// 点到直线的距离 = |叉积| / |直线向量|
	if (lineLength < 1e-6) return cv::norm(pointVec); // 直线退化为点

	return crossProduct / lineLength;
}


double inline cross_point_line(const cv::Point2f& point,
	const cv::Point2f& linePoint1,
	const cv::Point2f& linePoint2)
{
	// 计算向量
	cv::Point2f lineVec = linePoint2 - linePoint1;
	cv::Point2f pointVec = point - linePoint1;

	// 叉积的模 = |lineVec| * |pointVec| * sin(theta)
	return std::abs(lineVec.x * pointVec.y - lineVec.y * pointVec.x);
}

const double EPS = 1e-9;

// 判断浮点数是否接近0
bool isZero(double x) {
	return std::fabs(x) < EPS;
}

// 方法1：使用直线的一般式方程 Ax + By + C = 0
cv::Point2f lineIntersectionGeneral(const cv::Point& p1, const cv::Point& p2,
	const cv::Point& p3, const cv::Point& p4,
	bool& parallel) {
	// 直线1: A1*x + B1*y + C1 = 0
	double A1 = p2.y - p1.y;
	double B1 = p1.x - p2.x;
	double C1 = p2.x * p1.y - p1.x * p2.y;

	// 直线2: A2*x + B2*y + C2 = 0
	double A2 = p4.y - p3.y;
	double B2 = p3.x - p4.x;
	double C2 = p4.x * p3.y - p3.x * p4.y;

	// 计算行列式
	double det = A1 * B2 - A2 * B1;

	if (isZero(det)) {
		parallel = true;
		return cv::Point2f(0, 0);
	}

	parallel = false;
	double x = (B1 * C2 - B2 * C1) / det;
	double y = (C1 * A2 - C2 * A1) / det;

	return cv::Point2f(x, y);
}


cv::Point2f calc_rect_intersection(std::vector<cv::Point>& c1, int i, std::vector<cv::Point>& c2, int j)
{
	float max_i = 0;
	int max_idx = -1;
	for (int k = 0; k < 4; ++k)
	{
		if (cv::norm(c1[i] - c1[k]) > max_i)
		{
			max_idx = k;
			max_i = cv::norm(c1[i] - c1[k]);
		}
	}
	int jk[2]; int count = 0;
	for (int k = 0; k < 4; ++k)
	{
		if (k != max_idx && k != i)
		{
			jk[count++] = k;
		}
	}
	//std::cout << i << '\t' << jk[0] << '\t' << jk[1] << '\n';

	std::vector<cv::Point> c21 = c2, c22 = c2;
	std::sort(c21.begin(), c21.end(), [&](cv::Point& l, cv::Point& r) {
		return cross_point_line(l, c1[jk[0]], c1[i]) < cross_point_line(r, c1[jk[0]], c1[i]);
		});
	if (c21[0] == c2[j])
	{
		std::swap(c21[0], c21[1]);
	}
	std::sort(c22.begin(), c22.end(), [&](cv::Point& l, cv::Point& r) {
		return cross_point_line(l, c1[jk[1]], c1[i]) < cross_point_line(r, c1[jk[1]], c1[i]);
		});
	if (c22[0] == c2[j])
	{
		std::swap(c22[0], c22[1]);
	}
	bool parrallel;
	return lineIntersectionGeneral(c1[jk[0]], c21[0], c1[jk[1]], c22[0], parrallel);
}

/**
 * @brief 寻找n个离中心最近的棋盘格角点
 * @param img 输入图像，cv_8u
 * @param round_size 输入点的个数
 * @param points 输出点的坐标
 * @return 
*/
bool detect_corner(cv::Mat& img, int round_size, std::vector<cv::Point2f>& points)
{
	cv::Mat gray;
	if (img.type() != CV_8U)
	{
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	}
	else
	{
		gray = img;
	}
	typedef std::vector<cv::Point> cvContour;
	cv::Mat mask;
	int thresh = cv::threshold(gray, mask, -1, 255, cv::THRESH_OTSU);
	//分割阈值
	std::cout << "otsu threshold:\t" << thresh << std::endl;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	auto kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	cv::dilate(mask, mask, kernel);
	// cv::erode(mask, mask, kernel);
	cv::findContours(mask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
	int count = 0;
	for (int i = 0; i < contours.size(); ++i)
	{
		double area = cv::contourArea(contours[i]);
		if (hierarchy[i][3] == -1)continue;
		if (area <25 || area>gray.cols * gray.rows * 0.04)continue;
		contours[count++] = contours[i];
	}

	if (count <= round_size)
	{
		return false;
	}

	contours.resize(count);

	cv::Point2f center = cv::Point(gray.cols / 2, gray.rows / 2);

	std::vector<cvContour>cc(1), cc_round(round_size);
	find_nereast_rect(contours, cc, 1, center);

	cv::Point2f ccc = centerContour(cc[0]);
	find_nereast_rect(contours, cc_round, round_size, ccc);

	std::vector<cvContour>simples;
	cvContour ctemp;
	for (int i = 0; i < round_size; ++i)
	{
		if (cc_round[i].size() < 4) return false;
		if (approx_bounding(cc_round[i], ctemp))
		{
			simples.push_back(std::move(ctemp));
		}
	}

	if (cc[0].size() < 4) return false;

	if (approx_bounding(cc[0], ctemp))
	{
		simples.push_back(std::move(ctemp));
	}
	if (simples.size() <= round_size)
	{
		return false;
	}
	points.clear();
	count = 0;
	for (int i = 0; i < round_size + 1; ++i)
	{
		for (int m = 0; m < 4; ++m)
		{
			for (int j = i + 1; j < round_size + 1; ++j)
			{
				for (int k = 0; k < 4; ++k)
				{
					if (cv::norm(simples[j][k] - simples[i][m]) < 25)
					{
						//std::cout << m << k << std::endl;
						points.push_back(calc_rect_intersection(simples[i], m, simples[j], k));
					}
				}
			}
		}
	}
	if (points.size() < round_size)return false;
	std::nth_element(points.begin(), points.begin() + round_size, points.end(), [&](cv::Point2f& l, cv::Point2f& r)
		{
			return cv::norm(l - center) < cv::norm(r - center);
		});
	points.resize(round_size);

	cv::cornerSubPix(gray, points, cv::Size(5, 5), cv::Size(-1, -1),
		cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 15, 0.1));
	return true;
}

/*演示函数*/
int main_demo()
{
	auto img = cv::imread(R"(C:\Users\yommy.wei\Desktop\20260324-162432.bmp)", cv::IMREAD_GRAYSCALE);
	int round_size = 20;
	std::vector<cv::Point2f> points;
	detect_corner(img, round_size, points);
	cv::Mat result(img.size(), CV_8UC3);
	cv::cvtColor(img, result, cv::COLOR_GRAY2BGR);
	for (int i = 0; i < points.size(); ++i)
	{
		cv::drawMarker(result, points[i], cv::Scalar(rand() % 255, rand() % 255, rand() % 255), 0, 40, 12);
	}
	cv::imshow("mask", result);
	cv::waitKey();
	return 0;
}
