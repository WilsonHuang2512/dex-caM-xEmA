// precision_monitor.cpp
#define _CRT_SECURE_NO_WARNINGS
// Continuously measures 3D calibration board precision every N minutes.
// Between precision captures the camera runs continuously in projector-off mono
// mode at maximum exposure (model 3), then switches back to the full 3D
// parameters from camera_config.json for each measurement, then returns to mono.
//
// - Logs every result to precision_log_<date>.csv
// - Generates precision_chart_<date>.html every 6 hours and on Ctrl+C / exit
//
// Usage:
//   precision_monitor.exe                         <- all settings from camera_config.json
//   precision_monitor.exe 192.168.15.83           <- override ip
//   precision_monitor.exe 192.168.15.83 12        <- override ip + board mm
//   precision_monitor.exe 192.168.15.83 12 10     <- override ip + board + interval minutes
//
// JSON keys used (firmware section):
//   mono_exposure_time  - projector-off exposure in µs  (default: 100000)

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>
#include <csignal>
#include <ctime>

#include <opencv2/opencv.hpp>
#include "xcamera.h"
#include "enumerate.h"

using namespace XEMA;
using namespace std::chrono;

// ===========================================================================
// Signal handling for clean exit
// ===========================================================================
static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

// ===========================================================================
// Line-by-line section-aware JSON reader
// ===========================================================================
struct JsonData {
    std::map<std::string, std::map<std::string, std::string>>       vals;
    std::map<std::string, std::map<std::string, std::vector<int>>>  arrs;
};

static std::string trimStr(const std::string& s)
{
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

static JsonData parseJson(const std::string& path)
{
    JsonData jd;
    std::ifstream f(path);
    if (!f.is_open()) { std::cout << "[config] cannot open: " << path << "\n"; return jd; }

    std::string section, arr_key, line;
    bool in_array = false;
    std::vector<int> arr_vals;

    while (std::getline(f, line)) {
        std::string t = trimStr(line);
        if (t.empty() || t == "{" || t == "}" || t == "},") continue;

        if (in_array) {
            if (t == "]" || t == "],") {
                jd.arrs[section][arr_key] = arr_vals;
                in_array = false; continue;
            }
            std::string elem = t;
            if (!elem.empty() && elem.back() == ',') elem.pop_back();
            if (!elem.empty()) arr_vals.push_back(std::atoi(elem.c_str()));
            continue;
        }

        if (t.empty() || t[0] != '"') continue;
        size_t kend = t.find('"', 1);
        if (kend == std::string::npos) continue;
        std::string key = t.substr(1, kend - 1);
        size_t colon = t.find(':', kend + 1);
        if (colon == std::string::npos) continue;
        std::string val = trimStr(t.substr(colon + 1));
        if (!val.empty() && val.back() == ',') val.pop_back();
        val = trimStr(val);

        if (val == "{") { section = key; }
        else if (val == "[") { in_array = true; arr_key = key; arr_vals.clear(); }
        else if (!section.empty()) { jd.vals[section][key] = val; }
    }
    return jd;
}

static int jInt(const JsonData& jd, const std::string& sec,
    const std::string& key, int def = 0)
{
    auto si = jd.vals.find(sec); if (si == jd.vals.end()) return def;
    auto ki = si->second.find(key); if (ki == si->second.end()) return def;
    return std::atoi(ki->second.c_str());
}
static std::string jStr(const JsonData& jd, const std::string& sec,
    const std::string& key, const std::string& def = "")
{
    auto si = jd.vals.find(sec); if (si == jd.vals.end()) return def;
    auto ki = si->second.find(key); if (ki == si->second.end()) return def;
    std::string v = ki->second;
    if (v.size() >= 2 && v.front() == '"' && v.back() == '"') v = v.substr(1, v.size() - 2);
    return v;
}
static std::vector<int> jArr(const JsonData& jd, const std::string& sec, const std::string& key)
{
    auto si = jd.arrs.find(sec); if (si == jd.arrs.end()) return {};
    auto ki = si->second.find(key); if (ki == si->second.end()) return {};
    return ki->second;
}

// ===========================================================================
// Config
// ===========================================================================
struct Config {
    std::string ip = "";
    int calibration_board = 20;
    int exposure_model = 0;
    int engine = 1;
    int led_current = 1023;
    int camera_exposure_time = 10000;
    int camera_gain = 0;
    int confidence = 5;
    int mixed_exposure_num = 2;
    int mixed_exposure_param_list[6] = { 6000,30000,60000,60000,60000,60000 };
    int mixed_led_param_list[6] = { 100,1023,1023,1023,1023,1023 };
    int repetition_count = 3;
    // Mono (projector-off) continuous capture between precision measurements
    int mono_exposure_time = 100000;  // µs — projector-off brightness, default max
};

static Config loadConfig(const std::string& path)
{
    Config c;
    JsonData jd = parseJson(path);
    c.ip = jStr(jd, "gui", "ip", c.ip);
    c.calibration_board = jInt(jd, "gui", "calibration_board", c.calibration_board);
    c.exposure_model = jInt(jd, "gui", "exposure_model", c.exposure_model);
    c.engine = jInt(jd, "gui", "engine", c.engine);
    c.repetition_count = jInt(jd, "gui", "repetition_count", c.repetition_count);
    c.led_current = jInt(jd, "firmware", "led_current", c.led_current);
    c.camera_exposure_time = jInt(jd, "firmware", "camera_exposure_time", c.camera_exposure_time);
    c.camera_gain = jInt(jd, "firmware", "camera_gain", c.camera_gain);
    c.confidence = jInt(jd, "firmware", "confidence", c.confidence);
    c.mixed_exposure_num = jInt(jd, "firmware", "mixed_exposure_num", c.mixed_exposure_num);
    auto mep = jArr(jd, "firmware", "mixed_exposure_param_list");
    for (int i = 0; i < (int)mep.size() && i < 6; i++) c.mixed_exposure_param_list[i] = mep[i];
    auto mlp = jArr(jd, "firmware", "mixed_led_param_list");
    for (int i = 0; i < (int)mlp.size() && i < 6; i++) c.mixed_led_param_list[i] = mlp[i];
    c.mono_exposure_time = jInt(jd, "firmware", "mono_exposure_time", c.mono_exposure_time);
    std::cout << "[config] ip=" << c.ip
        << "  board=" << c.calibration_board << "mm"
        << "  exposure=" << c.camera_exposure_time << "us"
        << "  mono_exposure=" << c.mono_exposure_time << "us"
        << "  engine=" << c.engine << "\n";
    return c;
}

// ===========================================================================
// Board
// ===========================================================================
struct BoardConfig { int rows, cols, width, height; };
static BoardConfig getBoard(int mm)
{
    switch (mm) {
    case  4: return{ 11,7, 4, 2 };
    case 12: return{ 11,7,12, 6 };
    case 20: return{ 11,7,20,10 };
    case 40: return{ 11,7,40,20 };
    case 80: return{ 11,7,80,40 };
    default: std::cout << "unknown board, using 20mm\n"; return{ 11,7,20,10 };
    }
}
static std::vector<cv::Point3f> genWorld(const BoardConfig& b)
{
    std::vector<cv::Point3f> pts;
    for (int r = 0; r < b.rows; r++)
        for (int c = 0; c < b.cols; c++) {
            float x = (r % 2 == 0) ? b.width * (float)c : b.width * (float)c + 0.5f * b.width;
            pts.push_back(cv::Point3f(x, b.height * (float)r, 0.f));
        }
    return pts;
}

// ===========================================================================
// Geometry
// ===========================================================================
static bool bilinear(const float* pc, int W, int H, float px, float py, cv::Point3f& out)
{
    int x0 = (int)px, y0 = (int)py, x1 = x0 + 1, y1 = y0 + 1;
    if (x0 < 0 || y0 < 0 || x1 >= W || y1 >= H) return false;
    float fx = px - x0, fy = py - y0;
    auto at = [&](int r, int c) {return pc + (r * W + c) * 3; };
    const float* p00 = at(y0, x0), * p10 = at(y0, x1), * p01 = at(y1, x0), * p11 = at(y1, x1);
    if (p00[2] == 0 || p10[2] == 0 || p01[2] == 0 || p11[2] == 0) return false;
    float v[3];
    for (int i = 0; i < 3; i++)
        v[i] = p00[i] * (1 - fx) * (1 - fy) + p10[i] * fx * (1 - fy) + p01[i] * (1 - fx) * fy + p11[i] * fx * fy;
    out = cv::Point3f(v[0], v[1], v[2]);
    return true;
}

static void svdIcp(const cv::Mat& src, const cv::Mat& dst, cv::Mat& R, cv::Mat& t)
{
    cv::Mat sm, dm;
    cv::reduce(src, sm, 0, cv::REDUCE_AVG);
    cv::reduce(dst, dm, 0, cv::REDUCE_AVG);
    cv::Mat U, S, Vt;
    cv::SVD::compute((src - cv::repeat(sm, src.rows, 1)).t() * (dst - cv::repeat(dm, dst.rows, 1)), S, U, Vt);
    R = Vt.t() * U.t();
    if (cv::determinant(R) < 0) { Vt.row(2) *= -1; R = Vt.t() * U.t(); }
    t = dm.t() - R * sm.t();
}

static std::vector<cv::Point3f> applyTransform(const std::vector<cv::Point3f>& pts,
    const cv::Mat& R, const cv::Mat& t)
{
    std::vector<cv::Point3f> out;
    for (const auto& p : pts) {
        cv::Mat v = (cv::Mat_<double>(3, 1) << (double)p.x, (double)p.y, (double)p.z);
        cv::Mat r = R * v + t;
        out.push_back({ (float)r.at<double>(0),(float)r.at<double>(1),(float)r.at<double>(2) });
    }
    return out;
}

static double meanDist(const std::vector<cv::Point3f>& a, const std::vector<cv::Point3f>& b)
{
    double s = 0;
    for (size_t i = 0; i < a.size(); i++) {
        double dx = a[i].x - b[i].x, dy = a[i].y - b[i].y, dz = a[i].z - b[i].z;
        s += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return s / (double)a.size();
}

// ===========================================================================
// Timestamp helpers
// ===========================================================================
static std::string nowStr(const char* fmt = "%Y-%m-%d %H:%M:%S")
{
    time_t t = time(nullptr);
    char buf[64];
    strftime(buf, sizeof(buf), fmt, localtime(&t));
    return buf;
}
static std::string dateStr() { return nowStr("%Y%m%d_%H%M%S"); }

// ===========================================================================
// Result record
// ===========================================================================
struct Record {
    std::string timestamp;
    double precision_mm;
    int valid_points;
    bool detected;
};

// ===========================================================================
// CSV export
// ===========================================================================
static void writeCSV(const std::string& path, const std::vector<Record>& records)
{
    std::ofstream f(path);
    f << "timestamp,precision_mm,valid_points,detected\n";
    for (const auto& r : records) {
        f << r.timestamp << ","
            << (r.detected ? std::to_string(r.precision_mm) : "") << ","
            << r.valid_points << ","
            << (r.detected ? "yes" : "no") << "\n";
    }
    std::cout << "[csv] saved: " << path << "\n";
}

// ===========================================================================
// HTML chart export (self-contained, no internet required — inline Chart.js)
// Uses a CDN link; works offline if browser has cached it, or replace with
// a local copy of chart.js if needed.
// ===========================================================================
static void writeChart(const std::string& path, const std::vector<Record>& records)
{
    // Build JS arrays
    std::ostringstream labels, data_vals, colors;
    bool first = true;
    for (const auto& r : records) {
        if (!first) { labels << ","; data_vals << ","; colors << ","; }
        first = false;
        labels << "\"" << r.timestamp << "\"";
        if (r.detected)
            data_vals << r.precision_mm;
        else
            data_vals << "null";
        // color: green < 2mm, yellow < 5mm, red >= 5mm, grey = not detected
        if (!r.detected)        colors << "\"rgba(150,150,150,0.7)\"";
        else if (r.precision_mm < 2.0) colors << "\"rgba(40,167,69,0.85)\"";
        else if (r.precision_mm < 5.0) colors << "\"rgba(255,193,7,0.85)\"";
        else                           colors << "\"rgba(220,53,69,0.85)\"";
    }

    // Count stats
    int detected = 0; double sum = 0, mn = 1e9, mx = 0;
    for (const auto& r : records) if (r.detected) {
        detected++; sum += r.precision_mm;
        mn = std::min(mn, r.precision_mm);
        mx = std::max(mx, r.precision_mm);
    }
    double avg = detected ? sum / detected : 0;

    std::ofstream f(path);
    f << R"(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Precision Monitor</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  body { font-family: Arial, sans-serif; background: #1e1e2e; color: #cdd6f4; margin: 20px; }
  h1   { color: #89b4fa; }
  .stats { display: flex; gap: 20px; margin: 16px 0; flex-wrap: wrap; }
  .stat  { background: #313244; border-radius: 8px; padding: 12px 20px; min-width: 130px; }
  .stat .val { font-size: 1.8em; font-weight: bold; color: #89b4fa; }
  .stat .lbl { font-size: 0.85em; color: #a6adc8; }
  .chart-box { background: #313244; border-radius: 10px; padding: 20px; margin-top: 16px; }
  .ok   { color: #a6e3a1; } .warn { color: #f9e2af; } .bad { color: #f38ba8; }
</style>
</head>
<body>
<h1>&#x1F4CF; Precision Monitor Report</h1>
<p>Generated: )";
    f << nowStr() << " &nbsp;|&nbsp; Total measurements: " << records.size()
        << " &nbsp;|&nbsp; Board detected: " << detected << "/" << records.size() << "</p>\n";

    f << "<div class=\"stats\">\n";
    f << "  <div class=\"stat\"><div class=\"val\">" << std::fixed << std::setprecision(3) << avg << "</div><div class=\"lbl\">Avg (mm)</div></div>\n";
    f << "  <div class=\"stat\"><div class=\"val\">" << mn << "</div><div class=\"lbl\">Min (mm)</div></div>\n";
    f << "  <div class=\"stat\"><div class=\"val\">" << mx << "</div><div class=\"lbl\">Max (mm)</div></div>\n";
    f << "  <div class=\"stat\"><div class=\"val\">" << records.size() << "</div><div class=\"lbl\">Measurements</div></div>\n";
    f << "</div>\n";

    f << R"(
<div class="chart-box">
<canvas id="chart" height="90"></canvas>
</div>
<script>
const ctx = document.getElementById('chart').getContext('2d');
new Chart(ctx, {
  type: 'line',
  data: {
    labels: [)" << labels.str() << R"(],
    datasets: [{
      label: 'Precision (mm)',
      data: [)" << data_vals.str() << R"(],
      borderColor: 'rgba(137,180,250,0.9)',
      backgroundColor: [)" << colors.str() << R"(],
      borderWidth: 2,
      pointRadius: 5,
      pointBackgroundColor: [)" << colors.str() << R"(],
      tension: 0.3,
      spanGaps: false
    }]
  },
  options: {
    responsive: true,
    plugins: {
      legend: { labels: { color: '#cdd6f4' } },
      tooltip: {
        callbacks: {
          label: ctx => ctx.parsed.y !== null ? ctx.parsed.y.toFixed(3) + ' mm' : 'not detected'
        }
      }
    },
    scales: {
      x: { ticks: { color: '#a6adc8', maxRotation: 45 }, grid: { color: '#45475a' } },
      y: {
        ticks: { color: '#a6adc8' },
        grid: { color: '#45475a' },
        title: { display: true, text: 'Precision (mm)', color: '#a6adc8' },
        min: 0
      }
    }
  }
});
</script>
<p style="margin-top:20px;color:#585b70;font-size:0.8em;">
  Green &lt;2mm &nbsp; Yellow &lt;5mm &nbsp; Red &ge;5mm &nbsp; Grey = board not detected
</p>
</body></html>
)";

    std::cout << "[chart] saved: " << path << "\n";
}

// ===========================================================================
// One capture + precision measurement
// ===========================================================================
static Record measure(XCamera* cam, const Config& cfg,
    const BoardConfig& board, int W, int H,
    int capture_num)
{
    Record rec;
    rec.timestamp = nowStr();
    rec.detected = false;
    rec.precision_mm = 0;
    rec.valid_points = 0;

    // Capture
    char ts[30] = "";
    int ret = cam->captureData(capture_num, ts);
    if (ret != 0) {
        std::cout << "[" << rec.timestamp << "] capture failed (" << ret << ")\n";
        return rec;
    }

    // Retrieve data
    size_t bsz = (size_t)W * H, psz = (size_t)W * H * 3;
    unsigned char* bright = (unsigned char*)malloc(bsz);
    float* pc = (float*)malloc(psz * sizeof(float));
    memset(bright, 0, bsz); memset(pc, 0, psz * sizeof(float));
    cam->getBrightnessData(bright);
    cam->getPointcloudData(pc);

    // Invert + detect
    cv::Mat img(H, W, CV_8UC1, bright);
    cv::Mat inv; cv::bitwise_not(img, inv);

    cv::Size bsz2(board.cols, board.rows);
    std::vector<cv::Point2f> cpts;
    bool found = cv::findCirclesGrid(inv, bsz2, cpts,
        cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);

    if (!found) {
        std::cout << "[" << rec.timestamp << "] board not detected\n";
        free(bright); free(pc);
        return rec;
    }

    // 3D sampling
    std::vector<cv::Point3f> world_all = genWorld(board);
    std::vector<cv::Point3f> measured, world;
    for (size_t i = 0; i < cpts.size(); i++) {
        cv::Point3f pt3;
        if (bilinear(pc, W, H, cpts[i].x, cpts[i].y, pt3)) {
            measured.push_back(pt3);
            world.push_back(world_all[i]);
        }
    }
    free(bright); free(pc);

    rec.valid_points = (int)measured.size();
    if ((int)measured.size() < 4) {
        std::cout << "[" << rec.timestamp << "] too few valid 3D points (" << measured.size() << ")\n";
        return rec;
    }

    // ICP
    int N = (int)measured.size();
    cv::Mat sm(N, 3, CV_64F), dm(N, 3, CV_64F);
    for (int i = 0; i < N; i++) {
        sm.at<double>(i, 0) = measured[i].x; sm.at<double>(i, 1) = measured[i].y; sm.at<double>(i, 2) = measured[i].z;
        dm.at<double>(i, 0) = world[i].x;    dm.at<double>(i, 1) = world[i].y;    dm.at<double>(i, 2) = world[i].z;
    }
    cv::Mat R, t; svdIcp(sm, dm, R, t);
    auto aligned = applyTransform(measured, R, t);
    rec.precision_mm = meanDist(world, aligned);
    rec.detected = true;

    const char* tag = (rec.precision_mm < 2.0) ? "OK" : (rec.precision_mm < 5.0) ? "WARN" : "BAD";
    std::cout << "[" << rec.timestamp << "] precision: "
        << std::fixed << std::setprecision(3) << rec.precision_mm
        << " mm  pts=" << N << "  [" << tag << "]\n";
    return rec;
}

// ===========================================================================
// Exposure mode switching helpers
// ===========================================================================

// Switch to projector-off mono mode (max brightness capture).
// Uses exposure model 3 which disables the projector entirely.
static void applyMonoMode(XCamera* cam, const Config& cfg)
{
    cam->setParamGenerateBrightness(3, (float)cfg.mono_exposure_time);
    std::cout << "[mono] projector OFF, exposure=" << cfg.mono_exposure_time << " us\n";
}

// Restore the normal 3D capture parameters from cfg.
static void applyNormalMode(XCamera* cam, const Config& cfg, int& capture_num)
{
    cam->setParamLedCurrent(cfg.led_current);
    cam->setParamCameraExposure((float)cfg.camera_exposure_time);
    cam->setParamCameraGain((double)cfg.camera_gain);
    cam->setParamCameraConfidence((float)cfg.confidence);

    XemaEngine eng = (cfg.engine == 2) ? XemaEngine::Black :
        (cfg.engine == 1) ? XemaEngine::Reflect : XemaEngine::Normal;
    cam->setCaptureEngine(eng);

    capture_num = 1;
    if (cfg.exposure_model == 1) {
        capture_num = cfg.mixed_exposure_num;
        cam->setParamMultipleExposureModel(1);
        cam->setParamMixedHdr(capture_num,
            const_cast<int*>(cfg.mixed_exposure_param_list),
            const_cast<int*>(cfg.mixed_led_param_list));
    }
    else if (cfg.exposure_model == 2) {
        capture_num = cfg.repetition_count;
        cam->setParamMultipleExposureModel(2);
        cam->setParamRepetitionExposureNum(capture_num);
    }
    std::cout << "[normal] 3D params restored, exposure=" << cfg.camera_exposure_time << " us\n";
}

// Perform one projector-off mono brightness capture and discard the data.
// Returns true on success.
static bool captureMonoFrame(XCamera* cam, int W, int H)
{
    size_t bsz = (size_t)W * H;
    unsigned char* buf = (unsigned char*)malloc(bsz);
    int ret = cam->captureBrightnessData(buf, XemaColor::Gray);
    free(buf);
    if (ret != 0) {
        std::cout << "[mono] capture failed (" << ret << ")\n";
        return false;
    }
    return true;
}

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char* argv[])
{
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);

    // ------------------------------------------------------------------
    // Load config
    // ------------------------------------------------------------------
    Config cfg = loadConfig("camera_config.json");
    if (argc >= 2) cfg.ip = argv[1];
    if (argc >= 3) cfg.calibration_board = std::atoi(argv[2]);

    int interval_min = 20;
    if (argc >= 4) interval_min = std::atoi(argv[3]);

    if (cfg.ip.empty()) {
        std::cout << "ERROR: no ip in camera_config.json\n"; return 1;
    }

    BoardConfig board = getBoard(cfg.calibration_board);

    // Output file base names (include start date)
    std::string date_tag = dateStr();
    std::string csv_path = "precision_log_" + date_tag + ".csv";
    std::string chart_path = "precision_chart_" + date_tag + ".html";

    std::cout << "\n=== Precision Monitor ===\n"
        << "  interval      : " << interval_min << " min\n"
        << "  mono exposure : " << cfg.mono_exposure_time << " us (projector OFF)\n"
        << "  chart every   : 6 hr\n"
        << "  csv           : " << csv_path << "\n"
        << "  chart         : " << chart_path << "\n"
        << "  Press Ctrl+C to stop and export final chart.\n\n";

    // ------------------------------------------------------------------
    // Connect (stay connected for the whole session)
    // ------------------------------------------------------------------
    XCamera* cam = (XCamera*)createXCamera();
    std::cout << "connecting to " << cfg.ip << " ...\n";
    int ret = cam->connect(cfg.ip.c_str());
    if (ret != 0) {
        std::cout << "connect failed (" << ret << ")\n";
        destroyXCamera(cam); return 1;
    }
    std::cout << "connected\n";

    int W = 0, H = 0;
    cam->getCameraResolution(&W, &H);
    std::cout << "resolution " << W << "x" << H << "\n\n";

    // ------------------------------------------------------------------
    // Start in mono mode (projector off, max exposure) for continuous
    // background captures between precision measurements.
    // ------------------------------------------------------------------
    int capture_num = 1;  // will be set by applyNormalMode when needed
    applyMonoMode(cam, cfg);

    // ------------------------------------------------------------------
    // Measurement loop
    // ------------------------------------------------------------------
    std::vector<Record> records;
    auto last_chart = steady_clock::now();
    auto next_capture = steady_clock::now();   // first precision capture immediately
    const auto interval = minutes(interval_min);
    const auto chart_every = hours(6);

    while (g_running) {
        auto now = steady_clock::now();

        // ---- Time for a precision measurement? ----
        if (now >= next_capture) {
            // Switch to normal 3D mode
            applyNormalMode(cam, cfg, capture_num);

            Record rec = measure(cam, cfg, board, W, H, capture_num);
            records.push_back(rec);
            writeCSV(csv_path, records);
            next_capture = steady_clock::now() + interval;

            // Print next capture time
            time_t next_t = time(nullptr) + interval_min * 60;
            char buf[32]; strftime(buf, sizeof(buf), "%H:%M:%S", localtime(&next_t));
            std::cout << "  next precision capture at " << buf << "\n";

            // Return to mono mode for continuous background captures
            applyMonoMode(cam, cfg);
        }
        else {
            // ---- Mono frame (projector off, max exposure) ----
            captureMonoFrame(cam, W, H);
        }

        // ---- Time for a chart export? ----
        if (duration_cast<hours>(steady_clock::now() - last_chart) >= chart_every) {
            writeChart(chart_path, records);
            last_chart = steady_clock::now();
        }

        // Short sleep keeps Ctrl+C responsive without hammering the camera
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // ------------------------------------------------------------------
    // Final export on exit
    // ------------------------------------------------------------------
    std::cout << "\n[exit] stopping...\n";
    writeCSV(csv_path, records);
    writeChart(chart_path, records);

    cam->disconnect(cfg.ip.c_str());
    destroyXCamera(cam);
    std::cout << "[exit] done.  " << records.size() << " measurements recorded.\n";
    return 0;
}