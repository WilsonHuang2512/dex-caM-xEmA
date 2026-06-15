#include"DetectCCTag.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "cctag/Detection.hpp"
#include "cctag/utils/Exceptions.hpp"
#include "cctag/utils/FileDebug.hpp"
#include "cctag/utils/VisualDebug.hpp"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/exception/all.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/ptr_container/ptr_list.hpp>
#include <boost/timer/timer.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace cctag;


bool detectCCTag(cv::Mat img,std::vector<cv::Point2f>& point_list, std::vector<int>& id_list, std::vector<int>& status_list,int i)
{
    point_list.clear();
    id_list.clear();
    status_list.clear();

    const std::size_t nCrowns{ 3 };
    cctag::Parameters params(nCrowns);
    // if you want to use GPU
    //params.setUseCuda(true);

    // load the image e.g. from file   
    // choose a cuda pipe
    const int pipeId{ 0 }; 
    // an arbitrary id for the frame
    const int frameId{ 0 };

    // process the image
    boost::ptr_list<ICCTag> markers{};
    cctagDetection(markers, pipeId, frameId, img, params);

    for (auto it = markers.begin(); it != markers.end(); ++it)
    {

        cv::Point2f center = cv::Point2f(it->x(), it->y());
        int id = it->id();
        int status = it->getStatus();
        if (id >= 0 && id < 6) {
            point_list.push_back(center);
            id_list.push_back(id);
            status_list.push_back(status);
        }


    }

    if (0 == markers.size())
    {
        return false;
    }
    if (6 != point_list.size()) {

        return false;
    }
    return true;
}
 



//int main()
//{
//    std::cout << "Hello World!\n";
//    // set up the parameters
//    const std::size_t nCrowns{ 3 };
//    cctag::Parameters params(nCrowns);
//    // if you want to use GPU
//    params.setUseCuda(false);
//
//    // load the image e.g. from file
//    cv::Mat src = cv::imread("..\\..\\data\\cctag\\2.bmp");
//    cv::Mat graySrc;
//    cv::cvtColor(src, graySrc, cv::COLOR_BGR2GRAY);
//
//    // choose a cuda pipe
//    const int pipeId{ 0 };
//
//    // an arbitrary id for the frame
//    const int frameId{ 0 };
//
//    // process the image
//    boost::ptr_list<ICCTag> markers{};
//    cctagDetection(markers, pipeId, frameId, graySrc, params);
//
//    for (auto it = markers.begin(); it != markers.end(); ++it)
//    {
//        //ICCTag* ptr = *it;
//        // 使用指针进行操作 
//        cv::Point center = cv::Point(it->x(), it->y());
//
//        std::cout << "x: " << it->x() << "    ";
//        std::cout << "y: " << it->y() << "    ";
//        std::cout << "id: " << it->id() << std::endl;
//        //std::cout << "status: " << it->getStatus() << std::endl;
//
//        const int radius = 10;
//        const int fontSize = 3;
//        if (it->getStatus() == status::id_reliable)
//        {
//            std::cout << "success" << std::endl;
//            const cv::Scalar color = cv::Scalar(0, 255, 0, 255);
//            const auto rescaledOuterEllipse = it->rescaledOuterEllipse();
//
//            cv::circle(src, center, radius, color, 3);
//            cv::putText(src, std::to_string(it->id()), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, color, 3);
//            cv::ellipse(src,
//                center,
//                cv::Size(rescaledOuterEllipse.a(), rescaledOuterEllipse.b()),
//                rescaledOuterEllipse.angle() * 180 / boost::math::constants::pi<double>(),
//                0,
//                360,
//                color,
//                3);
//
//        }
//        else if (true)
//        {
//            std::cout << "failed" << std::endl;
//            const cv::Scalar color = cv::Scalar(0, 0, 255, 255);
//            cv::circle(src, center, radius, color, 2);
//            cv::putText(src, std::to_string(it->id()), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, color, 3);
//        }
//    }
//
//    std::cout << "find marker size: " << markers.size() << std::endl;
//    cv::namedWindow("result", cv::WINDOW_NORMAL);
//    cv::imshow("result", src);
//    cv::imwrite("results.bmp", src);
//    cv::waitKey(0);
//
//    std::cout << "detect finished!";
//}
//
// 