#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>
#include <opencv2/opencv.hpp>
#include <cctag/CCTag.hpp>

// ═══════════════════════════════════════════════════════════════════════
// Data Structures
// ═══════════════════════════════════════════════════════════════════════

struct Circle {
    cv::Point2f center;
    float radius;
    int id;
    int planeIndex;
};

struct Plane {
    float a, b, c, d;
    
    float distance(const cv::Point3f& pt) const {
        return std::abs(a * pt.x + b * pt.y + c * pt.z + d);
    }
};

struct PlaneRegion {
    cv::Rect rect;
    int index;
};

// ═══════════════════════════════════════════════════════════════════════
// Global Variables for Mouse Callbacks
// ═══════════════════════════════════════════════════════════════════════

struct MouseData {
    cv::Mat* image;
    std::vector<Circle>* circles;
    std::vector<PlaneRegion>* regions;
    cv::Point startPoint;
    bool drawing;
    int maxRegions;
};

// ═══════════════════════════════════════════════════════════════════════
// Circle Detection - EXACT SAME AS cctag_test
// ═══════════════════════════════════════════════════════════════════════

std::vector<Circle> detectCCTag(const cv::Mat& image) {
    std::vector<Circle> circles;
    
    std::cout << "Detecting CCTag markers..." << std::endl;
    std::cout << "  Original size: " << image.cols << "x" << image.rows << std::endl;
    
    // ═══════════════════════════════════════════════════════════════
    // DOWNSCALE for CCTag detection (save memory)
    // ═══════════════════════════════════════════════════════════════
    cv::Mat detectionImage = image;
    float scale = 1.0f;
    
    // If image is larger than 1000px, downscale it
    int maxDim = std::max(image.cols, image.rows);
    if (maxDim > 1000) {
        scale = 1000.0f / maxDim;
        cv::resize(image, detectionImage, cv::Size(), scale, scale, cv::INTER_LINEAR);
        std::cout << "  Downscaled to: " << detectionImage.cols << "x" 
                  << detectionImage.rows << " (scale=" << scale << ")" << std::endl;
    }
    
    int pipeId = 0;
    std::size_t frame = 0;
    cctag::Parameters params;
    boost::ptr_list<cctag::ICCTag> markers;
    
    try {
        cctag::cctagDetection(markers, pipeId, frame, detectionImage, params);
        
        std::cout << "CCTag detection completed!" << std::endl;
        std::cout << "  Total detected: " << markers.size() << std::endl;
        
        int validCount = 0;
        for (const auto& marker : markers) {
            if (marker.getStatus() == 1) {
                validCount++;
            }
        }
        std::cout << "  Valid markers (status=1): " << validCount << std::endl;
        
        int idx = 1;
        for (const auto& marker : markers) {
            if (marker.getStatus() == 1) {
                Circle c;
                
                // ═══════════════════════════════════════════════════════════
                // SCALE COORDINATES BACK to original size
                // ═══════════════════════════════════════════════════════════
                c.center = cv::Point2f(marker.x() / scale, marker.y() / scale);
                c.radius = 10.0f;
                c.id = marker.id();
                c.planeIndex = -1;
                circles.push_back(c);
                
                std::cout << "  Marker " << idx++ << ": "
                         << "Center=(" << c.center.x << ", " << c.center.y << "), "
                         << "ID=" << marker.id() << std::endl;
            }
        }
        
    } catch (const std::bad_alloc& e) {
        std::cerr << "CCTag memory error: " << e.what() << std::endl;
        std::cerr << "  Image too large for CCTag (" << detectionImage.cols << "x" 
                  << detectionImage.rows << ")" << std::endl;
        std::cerr << "  Try using a smaller image or manual selection" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "CCTag error: " << e.what() << std::endl;
    }
    
    return circles;
}

void mouseCallbackCircles(int event, int x, int y, int flags, void* userdata) {
    MouseData* data = static_cast<MouseData*>(userdata);
    
    if (event == cv::EVENT_LBUTTONDOWN) {
        Circle c;
        c.center = cv::Point2f(x, y);
        c.radius = 5.0f;
        c.id = data->circles->size() + 1;
        c.planeIndex = -1;
        data->circles->push_back(c);
        
        cv::circle(*data->image, cv::Point(x, y), 8, cv::Scalar(0, 255, 0), -1);
        cv::putText(*data->image, std::to_string(c.id),
                   cv::Point(x + 10, y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6,
                   cv::Scalar(255, 255, 0), 2);
        
        cv::imshow("Manual Circle Selection", *data->image);
        std::cout << "Circle " << c.id << " at (" << x << ", " << y << ")" << std::endl;
    }
}

std::vector<Circle> selectCirclesManually(const cv::Mat& image) {
    std::vector<Circle> circles;
    cv::Mat display = image.clone();
    if (display.channels() == 1) {
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
    }
    
    MouseData data;
    data.image = &display;
    data.circles = &circles;
    data.drawing = false;
    
    cv::namedWindow("Manual Circle Selection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Manual Circle Selection", 1200, 800);
    cv::setMouseCallback("Manual Circle Selection", mouseCallbackCircles, &data);
    
    std::cout << "\n=== Manual Circle Selection ===" << std::endl;
    std::cout << "Click on circle centers, press ENTER when done" << std::endl;
    
    cv::imshow("Manual Circle Selection", display);
    
    while (true) {
        int key = cv::waitKey(1);
        if (key == 13) break;
        else if (key == 27) { circles.clear(); break; }
    }
    
    cv::destroyWindow("Manual Circle Selection");
    return circles;
}

// ═══════════════════════════════════════════════════════════════════════
// Plane Region Selection
// ═══════════════════════════════════════════════════════════════════════

void mouseCallbackRegions(int event, int x, int y, int flags, void* userdata) {
    MouseData* data = static_cast<MouseData*>(userdata);
    
    if (event == cv::EVENT_LBUTTONDOWN && data->regions->size() < data->maxRegions) {
        data->startPoint = cv::Point(x, y);
        data->drawing = true;
    }
    else if (event == cv::EVENT_MOUSEMOVE && data->drawing) {
        cv::Mat temp = data->image->clone();
        
        for (const auto& r : *data->regions) {
            cv::Scalar color = r.index == 0 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
            cv::rectangle(temp, r.rect, color, 2);
        }
        
        cv::Scalar color = data->regions->size() == 0 ? 
                          cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
        cv::rectangle(temp, data->startPoint, cv::Point(x, y), color, 2);
        cv::imshow("Plane Region Selection", temp);
    }
    else if (event == cv::EVENT_LBUTTONUP && data->drawing) {
        data->drawing = false;
        
        PlaneRegion region;
        region.rect = cv::Rect(
            std::min(data->startPoint.x, x),
            std::min(data->startPoint.y, y),
            std::abs(x - data->startPoint.x),
            std::abs(y - data->startPoint.y)
        );
        region.index = data->regions->size();
        data->regions->push_back(region);
        
        cv::Scalar color = region.index == 0 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
        cv::rectangle(*data->image, region.rect, color, 2);
        cv::putText(*data->image, "Plane " + std::to_string(region.index + 1),
                   cv::Point(region.rect.x + 5, region.rect.y + 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
        
        cv::imshow("Plane Region Selection", *data->image);
        std::cout << "Region " << (region.index + 1) << " selected" << std::endl;
    }
}

std::vector<PlaneRegion> selectPlaneRegions(const cv::Mat& image, int numRegions = 2) {
    std::vector<PlaneRegion> regions;
    cv::Mat display = image.clone();
    if (display.channels() == 1) {
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
    }
    
    MouseData data;
    data.image = &display;
    data.regions = &regions;
    data.maxRegions = numRegions;
    data.drawing = false;
    
    cv::namedWindow("Plane Region Selection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Plane Region Selection", 1200, 800);
    cv::setMouseCallback("Plane Region Selection", mouseCallbackRegions, &data);
    
    std::cout << "\n=== Plane Region Selection ===" << std::endl;
    std::cout << "Draw 2 rectangles (Plane 1: BLUE, Plane 2: GREEN)" << std::endl;
    std::cout << "Press ENTER when done" << std::endl;
    
    cv::imshow("Plane Region Selection", display);
    
    while (regions.size() < numRegions) {
        int key = cv::waitKey(1);
        if (key == 27) { regions.clear(); break; }
    }
    
    cv::waitKey(1000);
    cv::destroyWindow("Plane Region Selection");
    return regions;
}

// ═══════════════════════════════════════════════════════════════════════
// Plane Fitting (RANSAC)
// ═══════════════════════════════════════════════════════════════════════

std::vector<cv::Point3f> extractPointsFromRegion(const PlaneRegion& region, const cv::Mat& depthMap) {
    std::vector<cv::Point3f> points;
    
    for (int y = region.rect.y; y < region.rect.y + region.rect.height; y++) {
        for (int x = region.rect.x; x < region.rect.x + region.rect.width; x++) {
            if (x < 0 || x >= depthMap.cols || y < 0 || y >= depthMap.rows) continue;
            
            float z = 0;
            if (depthMap.type() == CV_16UC1) {
                z = depthMap.at<unsigned short>(y, x);
            } else if (depthMap.type() == CV_32FC1) {
                z = depthMap.at<float>(y, x);
            }
            
            if (!std::isnan(z) && z > 0) {
                points.push_back(cv::Point3f(x, y, z));
            }
        }
    }
    
    return points;
}

Plane fitPlaneRANSAC(const std::vector<cv::Point3f>& points, int iterations = 1000, float threshold = 1.0f) {
    if (points.size() < 100) {
        std::cerr << "Not enough points: " << points.size() << std::endl;
        return {0, 0, 1, 0};
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);
    
    Plane bestPlane = {0, 0, 1, 0};
    int bestInliers = 0;
    
    for (int iter = 0; iter < iterations; iter++) {
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        int idx3 = dis(gen);
        
        if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) continue;
        
        const cv::Point3f& p1 = points[idx1];
        const cv::Point3f& p2 = points[idx2];
        const cv::Point3f& p3 = points[idx3];
        
        cv::Point3f v1 = p2 - p1;
        cv::Point3f v2 = p3 - p1;
        cv::Point3f normal = v1.cross(v2);
        
        float norm = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        if (norm < 1e-6) continue;
        
        normal.x /= norm;
        normal.y /= norm;
        normal.z /= norm;
        
        float d = -(normal.x * p1.x + normal.y * p1.y + normal.z * p1.z);
        
        int inliers = 0;
        for (const auto& pt : points) {
            float dist = std::abs(normal.x * pt.x + normal.y * pt.y + normal.z * pt.z + d);
            if (dist < threshold) inliers++;
        }
        
        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestPlane.a = normal.x;
            bestPlane.b = normal.y;
            bestPlane.c = normal.z;
            bestPlane.d = d;
        }
    }
    
    float inlierRatio = static_cast<float>(bestInliers) / points.size();
    std::cout << "  Inliers: " << bestInliers << "/" << points.size() 
              << " (" << (inlierRatio * 100) << "%)" << std::endl;
    
    return bestPlane;
}

// ═══════════════════════════════════════════════════════════════════════
// Results
// ═══════════════════════════════════════════════════════════════════════

void saveResults(const std::string& filename, const std::vector<Circle>& circles,
                const std::vector<Plane>& planes, const std::vector<cv::Point3f>& coords3d) {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    
    file << "=== XEMA Stereo Calibration Target Tag Results ===" << std::endl << std::endl;
    
    for (size_t i = 0; i < planes.size(); i++) {
        const auto& p = planes[i];
        file << "Plane " << (i + 1) << ": " << p.a << "x + " << p.b << "y + " 
             << p.c << "z + " << p.d << " = 0" << std::endl;
    }
    
    file << "\nCircle Centers and 3D Coordinates:" << std::endl;
    file << "ID\tPlane\t2D (x, y)\t\t3D (x, y, z)" << std::endl;
    
    for (size_t i = 0; i < circles.size() && i < coords3d.size(); i++) {
        file << circles[i].id << "\t" << (circles[i].planeIndex + 1) << "\t"
             << "(" << circles[i].center.x << ", " << circles[i].center.y << ")\t"
             << "(" << coords3d[i].x << ", " << coords3d[i].y << ", " << coords3d[i].z << ")" << std::endl;
    }
    
    file.close();
    std::cout << "\nResults saved to: " << filename << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════
// Calculate and Display Measurements
// ═══════════════════════════════════════════════════════════════════════

void calculateMeasurements(const std::vector<Circle>& circles, 
                          const std::vector<Plane>& planes,
                          const std::vector<cv::Point3f>& coords3d) {
    std::cout << "\n=== Measurements and Accuracy ===" << std::endl;
    
    // ═══════════════════════════════════════════════════════════════
    // 1. Distance Between Planes
    // ═══════════════════════════════════════════════════════════════
    if (planes.size() >= 2) {
        // Method 1: Using plane equations (perpendicular distance)
        // For parallel planes: distance = |d1 - d2| / |normal|
        float d1 = planes[0].d;
        float d2 = planes[1].d;
        
        // Normal vector magnitude (should be ~1 if normalized)
        float n1_mag = std::sqrt(planes[0].a * planes[0].a + 
                                 planes[0].b * planes[0].b + 
                                 planes[0].c * planes[0].c);
        float n2_mag = std::sqrt(planes[1].a * planes[1].a + 
                                 planes[1].b * planes[1].b + 
                                 planes[1].c * planes[1].c);
        
        float plane_distance = std::abs(d1 - d2);
        
        std::cout << "\n--- Plane Distance ---" << std::endl;
        std::cout << "  Plane 1 offset (d): " << d1 << " mm" << std::endl;
        std::cout << "  Plane 2 offset (d): " << d2 << " mm" << std::endl;
        std::cout << "  Distance between planes: " << plane_distance << " mm" << std::endl;
        std::cout << "  Distance in cm: " << (plane_distance / 10.0f) << " cm" << std::endl;
        
        // Expected value check
        float expected_distance = 700.0f; // 70 cm in mm
        float error = std::abs(plane_distance - expected_distance);
        float error_percent = (error / expected_distance) * 100.0f;
        
        std::cout << "\n  Expected distance: " << expected_distance << " mm (70 cm)" << std::endl;
        std::cout << "  Error: " << error << " mm (" << error_percent << "%)" << std::endl;
        
        if (error_percent < 5.0f) {
            std::cout << "  Accuracy: < 5% error" << std::endl;
        } else if (error_percent < 10.0f) {
            std::cout << "  Accuracy: GOOD < 10% error" << std::endl;
        } else {
            std::cout << "  Accuracy: POOR > 10% error" << std::endl;
        }
    }
    
    // ═══════════════════════════════════════════════════════════════
    // 2. Distances Between Markers
    // ═══════════════════════════════════════════════════════════════
    if (coords3d.size() >= 2) {
        std::cout << "\n--- Marker Distances ---" << std::endl;
        
        for (size_t i = 0; i < coords3d.size(); i++) {
            for (size_t j = i + 1; j < coords3d.size(); j++) {
                const cv::Point3f& p1 = coords3d[i];
                const cv::Point3f& p2 = coords3d[j];
                
                // 3D Euclidean distance
                float dx = p2.x - p1.x;
                float dy = p2.y - p1.y;
                float dz = p2.z - p1.z;
                
                float distance_3d = std::sqrt(dx*dx + dy*dy + dz*dz);
                float distance_2d = std::sqrt(dx*dx + dy*dy); // Horizontal only
                
                std::cout << "  Marker " << circles[i].id << " to Marker " << circles[j].id << ":" << std::endl;
                std::cout << "    3D distance: " << distance_3d << " mm (" << (distance_3d/10.0f) << " cm)" << std::endl;
                std::cout << "    2D distance (X-Y): " << distance_2d << " mm" << std::endl;
                std::cout << "    Depth difference (Z): " << std::abs(dz) << " mm" << std::endl;
            }
        }
    }
    
    // ═══════════════════════════════════════════════════════════════
    // 3. Plane Fit Quality
    // ═══════════════════════════════════════════════════════════════
    std::cout << "\n--- Plane Fit Quality ---" << std::endl;
    
    for (size_t i = 0; i < circles.size() && i < coords3d.size(); i++) {
        if (circles[i].planeIndex < 0 || circles[i].planeIndex >= planes.size()) continue;
        
        const Plane& plane = planes[circles[i].planeIndex];
        const cv::Point3f& pt = coords3d[i];
        
        // Calculate residual: how far the point is from the fitted plane
        float residual = plane.distance(pt);
        
        std::cout << "  Marker " << circles[i].id << " (Plane " << (circles[i].planeIndex + 1) << "):" << std::endl;
        std::cout << "    Residual: " << residual << " mm" << std::endl;
        
        if (residual < 1.0f) {
            std::cout << "    Fit quality: EXCELLENT (< 1mm)" << std::endl;
        } else if (residual < 5.0f) {
            std::cout << "    Fit quality: GOOD (< 5mm)" << std::endl;
        } else {
            std::cout << "    Fit quality: POOR (> 5mm)" << std::endl;
        }
    }
}

void visualizeResults(const cv::Mat& image, const std::vector<Circle>& circles,
                     const std::vector<PlaneRegion>& regions) {
    cv::Mat display = image.clone();
    if (display.channels() == 1) {
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
    }
    
    for (const auto& region : regions) {
        cv::Scalar color = region.index == 0 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
        cv::rectangle(display, region.rect, color, 2);
    }
    
    for (const auto& circle : circles) {
        cv::circle(display, circle.center, 8, cv::Scalar(0, 255, 0), -1);
        cv::putText(display, "ID:" + std::to_string(circle.id),
                   cv::Point(circle.center.x + 12, circle.center.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
    }
    
    cv::namedWindow("Final Results", cv::WINDOW_NORMAL);
    cv::resizeWindow("Final Results", 1200, 800);
    cv::imshow("Final Results", display);
    cv::imwrite("result_visualization.png", display);
    std::cout << "Press any key to close..." << std::endl;
    cv::waitKey(0);
}

// ═══════════════════════════════════════════════════════════════════════
// Load Circle Centers from File
// ═══════════════════════════════════════════════════════════════════════

std::vector<Circle> loadCirclesFromFile(const std::string& filename) {
    std::vector<Circle> circles;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open: " << filename << std::endl;
        return circles;
    }
    
    std::cout << "Loading circles from: " << filename << std::endl;
    
    std::string line;
    int count = 0;
    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        float x, y;
        int id;
        
        if (iss >> x >> y >> id) {
            Circle c;
            c.center = cv::Point2f(x, y);
            c.radius = 10.0f;
            c.id = id;
            c.planeIndex = -1;
            circles.push_back(c);
            
            count++;
            std::cout << "  Circle " << count << ": "
                     << "Center=(" << x << ", " << y << "), "
                     << "ID=" << id << std::endl;
        }
    }
    
    file.close();
    std::cout << "Loaded " << circles.size() << " circles from file" << std::endl;
    return circles;
}

// ═══════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    std::cout << "=== XEMA Stereo Calibration Target Tag (xsctt) ===" << std::endl;
    
    std::string brightPath = argc > 1 ? argv[1] : "data_bright.bmp";
    std::string depthPath = argc > 2 ? argv[2] : "data_depth_map.tiff";
    std::string circleFile = argc > 3 ? argv[3] : "";  // Optional circle file
    
    cv::Mat brightImage = cv::imread(brightPath, cv::IMREAD_GRAYSCALE);
    cv::Mat depthMap = cv::imread(depthPath, cv::IMREAD_UNCHANGED);
    
    if (brightImage.empty()) {
        std::cerr << "Cannot load: " << brightPath << std::endl;
        return -1;
    }
    if (depthMap.empty()) {
        std::cerr << "Cannot load: " << depthPath << std::endl;
        depthMap = cv::Mat(brightImage.size(), CV_16UC1, cv::Scalar(0));
    }
    
    // ═══════════════════════════════════════════════════════════
    // Step 1: Detect/Load Circles
    // ═══════════════════════════════════════════════════════════
    std::cout << "\n=== Step 1: Circle Detection ===" << std::endl;
    std::vector<Circle> circles;
    
    // Priority 1: Load from file if provided
    if (!circleFile.empty()) {
        std::cout << "Loading circles from file..." << std::endl;
        circles = loadCirclesFromFile(circleFile);
    }
    
    // Priority 2: Try CCTag detection if no file
    if (circles.empty()) {
        std::cout << "Attempting CCTag detection..." << std::endl;
        circles = detectCCTag(brightImage);
    }
    
    // Priority 3: Manual selection as fallback
    if (circles.empty()) {
        std::cout << "No circles loaded/detected. Manual selection..." << std::endl;
        circles = selectCirclesManually(brightImage);
    }
    
    if (circles.empty()) {
        std::cerr << "No circles!" << std::endl;
        return -1;
    }
    
    // Step 2: Select Plane Regions
    std::cout << "\n=== Step 2: Plane Region Selection ===" << std::endl;
    std::vector<PlaneRegion> regions = selectPlaneRegions(brightImage, 2);
    
    if (regions.size() != 2) {
        std::cerr << "Need 2 regions!" << std::endl;
        return -1;
    }
    
    // Step 3: Fit Planes
    std::cout << "\n=== Step 3: Plane Fitting ===" << std::endl;
    std::vector<Plane> planes;
    for (const auto& region : regions) {
        std::cout << "Fitting Plane " << (region.index + 1) << "..." << std::endl;
        auto points = extractPointsFromRegion(region, depthMap);
        std::cout << "  Points: " << points.size() << std::endl;
        
        if (points.size() < 100) continue;
        
        Plane plane = fitPlaneRANSAC(points);
        planes.push_back(plane);
        std::cout << "  Equation: " << plane.a << "x + " << plane.b << "y + " 
                  << plane.c << "z + " << plane.d << " = 0" << std::endl;
    }
    
    if (planes.size() != 2) {
        std::cerr << "Failed to fit 2 planes!" << std::endl;
        return -1;
    }
    
    // Step 4: Find 3D Coordinates
    std::cout << "\n=== Step 4: 3D Coordinates ===" << std::endl;
    std::vector<cv::Point3f> coords3d;
    
    for (auto& circle : circles) {
        int cx = cvRound(circle.center.x);
        int cy = cvRound(circle.center.y);
        
        for (const auto& region : regions) {
            if (region.rect.contains(cv::Point(cx, cy))) {
                circle.planeIndex = region.index;
                break;
            }
        }
        
        if (circle.planeIndex < 0) continue;
        
        float z = 0;
        if (cy >= 0 && cy < depthMap.rows && cx >= 0 && cx < depthMap.cols) {
            if (depthMap.type() == CV_16UC1) {
                z = depthMap.at<unsigned short>(cy, cx);
            }
        }
        
        if (z == 0 || std::isnan(z)) {
            const Plane& plane = planes[circle.planeIndex];
            if (std::abs(plane.c) > 1e-6) {
                z = -(plane.a * cx + plane.b * cy + plane.d) / plane.c;
            }
        }
        
        cv::Point3f coord3d(cx, cy, z);
        coords3d.push_back(coord3d);
        
        std::cout << "Circle " << circle.id << " (Plane " << (circle.planeIndex + 1) << "): "
                  << "3D=(" << coord3d.x << "," << coord3d.y << "," << coord3d.z << ")" << std::endl;
    }
    
    // Save and Visualize
    saveResults("calibration_results.txt", circles, planes, coords3d);
    
    // ═══════════════════════════════════════════════════════════════
    // ADD THIS LINE:
    calculateMeasurements(circles, planes, coords3d);
    // ═══════════════════════════════════════════════════════════════
    
    visualizeResults(brightImage, circles, regions);
    
    return 0;
}