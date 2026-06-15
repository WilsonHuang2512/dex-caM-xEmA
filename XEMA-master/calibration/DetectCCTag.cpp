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
 
