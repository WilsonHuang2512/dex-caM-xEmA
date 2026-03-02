#include <iostream>
#include <vector>
#include <cctag/CCTag.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    std::cout << "=== CCTag Detection Test ===" << std::endl;
    
    // Load brightness image
    std::string image_path = "data_bright.bmp";
    if (argc > 1) {
        image_path = argv[1];
    }
    
    std::cout << "Loading image: " << image_path << std::endl;
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    
    if (image.empty()) {
        std::cerr << "ERROR: Could not load image: " << image_path << std::endl;
        std::cerr << "Usage: cctag_test.exe [image_path]" << std::endl;
        return -1;
    }
    
    std::cout << "Image loaded successfully!" << std::endl;
    std::cout << "  Size: " << image.cols << "x" << image.rows << std::endl;
    
    // Display intensity range
    double minVal, maxVal;
    cv::minMaxLoc(image, &minVal, &maxVal);
    std::cout << "  Intensity range: [" << minVal << ", " << maxVal << "]" << std::endl;
    
    // Detect CCTag markers
    std::cout << "\nDetecting CCTag markers..." << std::endl;
    
    // CCTag parameters
    int pipeId = 0;           // CUDA pipeline ID (use 0 for single instance)
    std::size_t frame = 0;    // Frame number (can be anything)
    cctag::Parameters params; // Detection parameters
    
    // Output: list of detected markers
    boost::ptr_list<cctag::ICCTag> markers;
    
    try {
        // Call cctagDetection with correct signature
        cctag::cctagDetection(
            markers,      // output: detected markers
            pipeId,       // CUDA pipeline ID
            frame,        // frame number
            image,        // input image
            params        // detection parameters
            // durations and pBank are optional (nullptr by default)
        );
        
        std::cout << "CCTag detection completed!" << std::endl;
        std::cout << "  Total detected: " << markers.size() << std::endl;
        
        // Count valid markers (status == 1)
        int validCount = 0;
        for (const auto& marker : markers) {
            if (marker.getStatus() == 1) {
                validCount++;
            }
        }
        std::cout << "  Valid markers (status=1): " << validCount << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "CCTag detection error: " << e.what() << std::endl;
        return -1;
    }
    
    // Print marker details (only valid ones)
    if (!markers.empty()) {
        std::cout << "\n=== Detected Markers ===" << std::endl;
        
        int idx = 1;
        for (const auto& marker : markers) {
            // Only show valid markers (status == 1)
            if (marker.getStatus() == 1) {
                std::cout << "\nMarker " << idx++ << ":" << std::endl;
                std::cout << "  Center: (" << marker.x() << ", " << marker.y() << ")" << std::endl;
                std::cout << "  ID: " << marker.id() << std::endl;
                std::cout << "  Status: " << marker.getStatus() << std::endl;
            }
        }
    } else {
        std::cout << "\nNo CCTag markers detected!" << std::endl;
        std::cout << "This could mean:" << std::endl;
        std::cout << "  1. No CCTag markers in the image" << std::endl;
        std::cout << "  2. Your calibration target uses simple circles (not CCTag)" << std::endl;
        std::cout << "\nTrying Hough Circle detection as fallback..." << std::endl;
        
        // Fallback: Hough circles
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(image, circles, cv::HOUGH_GRADIENT,
                        1,      // dp
                        20,     // minDist
                        50,     // param1
                        30,     // param2
                        5,      // minRadius
                        100);   // maxRadius
        
        std::cout << "Hough detected " << circles.size() << " circles" << std::endl;
        
        if (!circles.empty()) {
            std::cout << "\n=== Detected Circles (Hough) ===" << std::endl;
            for (size_t i = 0; i < circles.size(); i++) {
                std::cout << "Circle " << i+1 << ": "
                         << "center=(" << circles[i][0] << ", " << circles[i][1] << "), "
                         << "radius=" << circles[i][2] << std::endl;
            }
        }
    }
    
    // Visualize results
    std::cout << "\nCreating visualization..." << std::endl;
    cv::Mat vis_image;
    cv::cvtColor(image, vis_image, cv::COLOR_GRAY2BGR);
    
    // Draw CCTag markers in green (only valid ones)
    int markerCount = 0;
    for (const auto& marker : markers) {
        if (marker.getStatus() == 1) {  // Only draw valid markers
            cv::Point center(cvRound(marker.x()), cvRound(marker.y()));
            
            // Draw center point
            cv::circle(vis_image, center, 8, cv::Scalar(0, 255, 0), -1);
            
            // Draw marker ID
            cv::putText(vis_image, 
                       "ID:" + std::to_string(marker.id()),
                       cv::Point(center.x + 15, center.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.7, 
                       cv::Scalar(0, 255, 0), 
                       2);
            
            markerCount++;
        }
    }
    
    // If no valid CCTag, draw Hough circles in yellow
    if (markerCount == 0) {
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(image, circles, cv::HOUGH_GRADIENT,
                        1, 20, 50, 30, 5, 100);
        
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            
            // Draw circle outline
            cv::circle(vis_image, center, radius, cv::Scalar(0, 255, 255), 2);
            // Draw center
            cv::circle(vis_image, center, 5, cv::Scalar(0, 255, 255), -1);
            // Draw number
            cv::putText(vis_image,
                       "#" + std::to_string(i+1),
                       cv::Point(center.x + 10, center.y),
                       cv::FONT_HERSHEY_SIMPLEX,
                       0.6,
                       cv::Scalar(255, 255, 0),
                       2);
        }
    }
    
    // Save result
    std::string output_path = "cctag_result.png";
    cv::imwrite(output_path, vis_image);
    std::cout << "Result saved to: " << output_path << std::endl;

    // Save marker coordinates to file
    if (markerCount > 0) {
        std::string coordFile = "cctag_markers.txt";
        std::ofstream out(coordFile);
        if (out.is_open()) {
            out << "# CCTag Marker Coordinates" << std::endl;
            out << "# x y id" << std::endl;
            
            for (const auto& marker : markers) {
                if (marker.getStatus() == 1) {
                    out << marker.x() << " " << marker.y() << " " << marker.id() << std::endl;
                }
            }
            
            out.close();
            std::cout << "\nMarker coordinates saved to: " << coordFile << std::endl;
            std::cout << "Use this file with: xsctt.exe data_bright.bmp data_depth_map.tiff cctag_markers.txt" << std::endl;
        }
    }
    
    // Show window
    cv::namedWindow("CCTag Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("CCTag Detection", 1200, 800);
    cv::imshow("CCTag Detection", vis_image);
    std::cout << "\nPress any key in the image window to exit..." << std::endl;
    cv::waitKey(0);
    
    return 0;
}