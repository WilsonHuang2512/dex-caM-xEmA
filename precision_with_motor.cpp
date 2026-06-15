// precision_servo_test.cpp
#define _CRT_SECURE_NO_WARNINGS
// Moves a servo/linear actuator UP and DOWN via Modbus RTU serial,
// measures 3D calibration board precision at each position, and logs results.
//
// Workflow per cycle:
//   1. Send UP command -> wait stay_time seconds -> capture + measure precision
//   2. Send DOWN command -> wait stay_time seconds -> capture + measure precision
//   3. Repeat until Ctrl+C
//
// Outputs:
//   precision_log_<date>.csv        -- all records (cycle, position, precision)
//   precision_chart_<date>.html     -- interactive chart, two datasets (Up/Down)
//
// Usage:
//   precision_servo_test.exe                              <- all from camera_config.json
//   precision_servo_test.exe 192.168.15.83                <- override camera IP
//   precision_servo_test.exe 192.168.15.83 20             <- override IP + board mm
//   precision_servo_test.exe 192.168.15.83 20 COM3        <- override IP + board + serial port
//   precision_servo_test.exe 192.168.15.83 20 COM3 3      <- + stay_time seconds

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
#include <cstdint>

#include <windows.h>

#include <opencv2/opencv.hpp>
#include "xcamera.h"
#include "enumerate.h"

using namespace XEMA;
using namespace std::chrono;

// ===========================================================================
// Signal handling
// ===========================================================================
static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

// ===========================================================================
// Modbus RTU serial port (Windows)
// ===========================================================================

// Modbus CRC16 (polynomial 0xA001, init 0xFFFF)
static uint16_t modbusCRC(const uint8_t* data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else               crc >>= 1;
        }
    }
    return crc;
}

// Append Modbus CRC to command and return full frame
static std::vector<uint8_t> modbusFrame(const uint8_t* data, int len)
{
    std::vector<uint8_t> frame(data, data + len);
    uint16_t crc = modbusCRC(data, len);
    frame.push_back((uint8_t)(crc & 0xFF));
    frame.push_back((uint8_t)((crc >> 8) & 0xFF));
    return frame;
}

// Open serial port; returns INVALID_HANDLE_VALUE on failure
static HANDLE openSerial(const std::string& port, int baud)
{
    std::string name = "\\\\.\\" + port;  // handle COM10+
    HANDLE h = CreateFileA(name.c_str(),
        GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) return INVALID_HANDLE_VALUE;

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(h, &dcb);

    COMMTIMEOUTS to = {};
    to.ReadIntervalTimeout = 50;
    to.ReadTotalTimeoutConstant = 500;
    to.ReadTotalTimeoutMultiplier = 10;
    to.WriteTotalTimeoutConstant = 500;
    to.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(h, &to);

    return h;
}

static bool sendSerial(HANDLE h, const std::vector<uint8_t>& frame)
{
    DWORD written = 0;
    return WriteFile(h, frame.data(), (DWORD)frame.size(), &written, NULL)
        && written == (DWORD)frame.size();
}

// ===========================================================================
// Servo commands (raw bytes before CRC)
// ===========================================================================
static const uint8_t RAW_UP[] = { 0x01, 0x06, 0x01, 0x07, 0x00, 0x01 };
static const uint8_t RAW_DOWN[] = { 0x01, 0x06, 0x01, 0x07, 0xFF, 0xFF };

// ===========================================================================
// Line-by-line JSON reader (unchanged from original)
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
// Config  (adds com_port, stay_time_sec, baud_rate vs original)
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
    // --- servo additions ---
    std::string com_port = "";   // e.g. "COM3"
    int baud_rate = 115200;
    int stay_time_sec = 3;    // wait after each move before capturing
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
    // servo params from "servo" section in camera_config.json (optional)
    c.com_port = jStr(jd, "servo", "com_port", c.com_port);
    c.baud_rate = jInt(jd, "servo", "baud_rate", c.baud_rate);
    c.stay_time_sec = jInt(jd, "servo", "stay_time_sec", c.stay_time_sec);

    std::cout << "[config] ip=" << c.ip
        << "  board=" << c.calibration_board << "mm"
        << "  exposure=" << c.camera_exposure_time << "us"
        << "  engine=" << c.engine
        << "  com=" << (c.com_port.empty() ? "(none)" : c.com_port)
        << "  stay=" << c.stay_time_sec << "s\n";
    return c;
}

// ===========================================================================
// Board helpers (unchanged)
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
// Geometry helpers (unchanged)
// ===========================================================================
static bool bilinear(const float* pc, int W, int H, float px, float py, cv::Point3f& out)
{
    int x0 = (int)px, y0 = (int)py, x1 = x0 + 1, y1 = y0 + 1;
    if (x0 < 0 || y0 < 0 || x1 >= W || y1 >= H) return false;
    float fx = px - x0, fy = py - y0;
    auto at = [&](int r, int c) { return pc + (r * W + c) * 3; };
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
        out.push_back({ (float)r.at<double>(0), (float)r.at<double>(1), (float)r.at<double>(2) });
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
// Timestamp helpers (unchanged)
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
// Record  -- adds cycle + position vs original
// ===========================================================================
struct Record {
    std::string timestamp;
    double      precision_mm = 0;
    int         valid_points = 0;
    bool        detected = false;
    int         cycle = 0;       // 1-based
    std::string position;                // "up" or "down"
};

// ===========================================================================
// CSV export  -- adds cycle + position columns
// ===========================================================================
static void writeCSV(const std::string& path, const std::vector<Record>& records)
{
    std::ofstream f(path);
    f << "timestamp,cycle,position,precision_mm,valid_points,detected\n";
    for (const auto& r : records) {
        f << r.timestamp << ","
            << r.cycle << ","
            << r.position << ","
            << (r.detected ? std::to_string(r.precision_mm) : "") << ","
            << r.valid_points << ","
            << (r.detected ? "yes" : "no") << "\n";
    }
    std::cout << "[csv] saved: " << path << "\n";
}

// ===========================================================================
// HTML chart -- 3-panel:
//   Top    : binned min/max/avg band (every BIN_SIZE cycles)
//   Middle : full sparkline, drag to brush a range
//   Bottom : detail view of brushed range
// ===========================================================================
static void writeChart(const std::string& path, const std::vector<Record>& records)
{
    if (records.empty()) { std::cout << "[chart] no data yet\n"; return; }

    // Find max cycle
    int max_cycle = 0;
    for (const auto& r : records)
        if (r.cycle > max_cycle) max_cycle = r.cycle;

    // Build per-cycle arrays (index = cycle-1)
    std::vector<double> up_vals(max_cycle, -1.0), down_vals(max_cycle, -1.0);
    for (const auto& r : records) {
        if (!r.detected || r.cycle < 1 || r.cycle > max_cycle) continue;
        if (r.position == "up")   up_vals[r.cycle - 1] = r.precision_mm;
        if (r.position == "down") down_vals[r.cycle - 1] = r.precision_mm;
    }

    // JS arrays  (null for undetected)
    std::ostringstream up_js, down_js;
    for (int i = 0; i < max_cycle; i++) {
        if (i) { up_js << ","; down_js << ","; }
        if (up_vals[i] >= 0) up_js << up_vals[i];   else up_js << "null";
        if (down_vals[i] >= 0) down_js << down_vals[i]; else down_js << "null";
    }

    // Stats (C++14 compatible)
    auto calcStats = [](const std::vector<double>& vals,
        int& cnt, double& avg, double& mn, double& mx) {
            cnt = 0; double sum = 0; mn = 1e9; mx = 0;
            for (double v : vals) if (v >= 0) { cnt++; sum += v; mn = std::min(mn, v); mx = std::max(mx, v); }
            avg = cnt ? sum / cnt : 0.0;
            if (!cnt) mn = 0.0;
    };
    int u_cnt = 0, d_cnt = 0;
    double u_avg = 0, u_min = 0, u_max = 0, d_avg = 0, d_min = 0, d_max = 0;
    calcStats(up_vals, u_cnt, u_avg, u_min, u_max);
    calcStats(down_vals, d_cnt, d_avg, d_min, d_max);

    std::ofstream f(path);

    // ---- HTML head ----
    f << R"ENDHTML(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Servo Precision Test</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  *    { box-sizing: border-box; }
  body { font-family: Arial, sans-serif; background: #1e1e2e; color: #cdd6f4; margin: 20px; }
  h1   { color: #89b4fa; margin-bottom: 4px; }
  .stats { display: flex; gap: 12px; margin: 14px 0; flex-wrap: wrap; }
  .stat  { background: #313244; border-radius: 8px; padding: 8px 16px; min-width: 100px; }
  .stat .val { font-size: 1.5em; font-weight: bold; }
  .stat .lbl { font-size: 0.78em; color: #a6adc8; }
  .up   { color: #89b4fa; }
  .down { color: #fab387; }
  .chart-box   { background: #313244; border-radius: 10px; padding: 14px 16px; margin-top: 10px; }
  .chart-title { font-size: 0.82em; color: #a6adc8; margin-bottom: 6px; letter-spacing: 0.03em; }
  #spark-wrap  { position: relative; cursor: crosshair; user-select: none; }
  #brush-info  { font-size: 0.78em; color: #89b4fa; margin-top: 4px; min-height: 1.1em; }
  #detail-title{ font-size: 0.82em; color: #a6adc8; margin-bottom: 6px; }
  .legend-row  { display:flex; gap:16px; font-size:0.8em; margin-top:10px; color:#a6adc8; flex-wrap:wrap; }
  .leg         { display:flex; align-items:center; gap:5px; }
  .leg-line    { width:24px; height:3px; border-radius:2px; display:inline-block; }
</style>
</head>
<body>
<h1>&#x1F4CF; Servo Precision Test</h1>
)ENDHTML";

    f << "<p style='margin:0 0 10px;color:#a6adc8;font-size:0.85em'>Generated: "
        << nowStr() << " &nbsp;|&nbsp; Total cycles: " << max_cycle << "</p>\n";

    // Stats cards
    f << "<div class=\"stats\">\n";
    auto card = [&](const std::string& css, const std::string& pos,
        int cnt, double avg, double mn, double mx) {
            f << std::fixed << std::setprecision(3);
            f << "  <div class='stat'><div class='val " << css << "'>" << pos << "</div><div class='lbl'>Position</div></div>\n";
            f << "  <div class='stat'><div class='val " << css << "'>" << avg << "</div><div class='lbl'>Avg (mm)</div></div>\n";
            f << "  <div class='stat'><div class='val " << css << "'>" << mn << "</div><div class='lbl'>Min (mm)</div></div>\n";
            f << "  <div class='stat'><div class='val " << css << "'>" << mx << "</div><div class='lbl'>Max (mm)</div></div>\n";
            f << "  <div class='stat'><div class='val " << css << "'>" << cnt << "</div><div class='lbl'>Detections</div></div>\n";
    };
    card("up", "UP", u_cnt, u_avg, u_min, u_max);
    card("down", "DOWN", d_cnt, d_avg, d_min, d_max);
    f << "</div>\n";

    // ---- Chart containers ----
    f << R"ENDHTML(
<div class="chart-box">
  <div class="chart-title">&#9632; OVERVIEW &mdash; binned min / avg / max every 50 cycles</div>
  <canvas id="chart-bins" height="75"></canvas>
</div>

<div class="chart-box">
  <div class="chart-title">&#9632; ALL CYCLES &mdash; drag to select a range &darr;</div>
  <div id="spark-wrap">
    <canvas id="chart-spark" height="38"></canvas>
  </div>
  <div id="brush-info">Drag on the sparkline to zoom into a range in the detail view below</div>
</div>

<div class="chart-box">
  <div id="detail-title" class="chart-title">&#9632; DETAIL VIEW &mdash; all cycles</div>
  <canvas id="chart-detail" height="95"></canvas>
</div>

<div class="legend-row">
  <span class="leg"><span class="leg-line" style="background:#89b4fa"></span> Up position</span>
  <span class="leg"><span class="leg-line" style="background:#fab387"></span> Down position</span>
  <span class="leg"><span class="leg-line" style="background:#40a769;height:8px;border-radius:2px"></span> &lt;2 mm OK</span>
  <span class="leg"><span class="leg-line" style="background:#ffc107;height:8px;border-radius:2px"></span> 2-5 mm WARN</span>
  <span class="leg"><span class="leg-line" style="background:#dc3545;height:8px;border-radius:2px"></span> &ge;5 mm BAD</span>
</div>
)ENDHTML";

    // ---- Inject data + JS ----
    f << "\n<script>\n";
    f << "const UP_VALS   = [" << up_js.str() << "];\n";
    f << "const DOWN_VALS = [" << down_js.str() << "];\n";
    f << "const MAX_CYCLE = " << max_cycle << ";\n";
    f << "const BIN_SIZE  = 50;\n\n";

    f << R"ENDHTML(
// ---- Shared theme ----
const CUP       = 'rgba(137,180,250,0.9)';
const CUP_BAND  = 'rgba(137,180,250,0.18)';
const CDOWN     = 'rgba(250,179,135,0.9)';
const CDOWN_BAND= 'rgba(250,179,135,0.18)';
const CGRID     = '#45475a';
const CTICK     = '#a6adc8';

function ptColor(v) {
  if (v === null) return 'rgba(150,150,150,0.5)';
  if (v < 2.0)   return 'rgba(64,167,105,0.9)';
  if (v < 5.0)   return 'rgba(255,193,7,0.9)';
  return                 'rgba(220,53,69,0.9)';
}

// ---- Build bins ----
function buildBins(binSize) {
  const n = Math.ceil(MAX_CYCLE / binSize);
  const bins = [];
  for (let b = 0; b < n; b++) {
    const s = b * binSize, e = Math.min((b+1)*binSize, MAX_CYCLE);
    let uMn=Infinity,uMx=-Infinity,uSum=0,uN=0;
    let dMn=Infinity,dMx=-Infinity,dSum=0,dN=0;
    for (let i = s; i < e; i++) {
      const u = UP_VALS[i], d = DOWN_VALS[i];
      if (u !== null) { uMn=Math.min(uMn,u); uMx=Math.max(uMx,u); uSum+=u; uN++; }
      if (d !== null) { dMn=Math.min(dMn,d); dMx=Math.max(dMx,d); dSum+=d; dN++; }
    }
    const lbl = (e-s===1) ? String(s+1) : (s+1)+'-'+e;
    bins.push({
      lbl,
      uMn: uN?uMn:null, uMx: uN?uMx:null, uAvg: uN?uSum/uN:null,
      dMn: dN?dMn:null, dMx: dN?dMx:null, dAvg: dN?dSum/dN:null,
    });
  }
  return bins;
}

// ==================================================================
// TOP CHART: binned overview with min/max band + avg line
// ==================================================================
const bins = buildBins(BIN_SIZE);

const chartBins = new Chart(document.getElementById('chart-bins'), {
  type: 'line',
  data: {
    labels: bins.map(b => b.lbl),
    datasets: [
      // UP band (max filled to min dataset below it)
      { label:'_uMax', data:bins.map(b=>b.uMx), borderColor:'transparent', backgroundColor:CUP_BAND,  fill:'+1', pointRadius:0, tension:0.3, order:4 },
      { label:'_uMin', data:bins.map(b=>b.uMn), borderColor:'transparent', backgroundColor:'transparent', fill:false, pointRadius:0, tension:0.3, order:5 },
      { label:'Up Avg',data:bins.map(b=>b.uAvg),borderColor:CUP,           backgroundColor:'transparent', fill:false, pointRadius:3, borderWidth:2, tension:0.3, order:2 },
      // DOWN band
      { label:'_dMax', data:bins.map(b=>b.dMx), borderColor:'transparent', backgroundColor:CDOWN_BAND, fill:'+1', pointRadius:0, tension:0.3, order:6 },
      { label:'_dMin', data:bins.map(b=>b.dMn), borderColor:'transparent', backgroundColor:'transparent', fill:false, pointRadius:0, tension:0.3, order:7 },
      { label:'Down Avg',data:bins.map(b=>b.dAvg),borderColor:CDOWN,       backgroundColor:'transparent', fill:false, pointRadius:3, borderWidth:2, tension:0.3, order:3 },
    ]
  },
  options: {
    animation: false, responsive: true,
    plugins: {
      legend: {
        labels: { color:CTICK, filter: item => !item.text.startsWith('_') }
      },
      tooltip: {
        filter: item => !item.dataset.label.startsWith('_'),
        callbacks: {
          label: function(ctx) {
            const b = bins[ctx.dataIndex];
            const isUp = ctx.dataset.label === 'Up Avg';
            const mn = isUp ? b.uMn : b.dMn;
            const mx = isUp ? b.uMx : b.dMx;
            const av = isUp ? b.uAvg : b.dAvg;
            if (av === null) return ctx.dataset.label + ': no data';
            return ctx.dataset.label + '  avg:' + av.toFixed(3) + '  min:' + mn.toFixed(3) + '  max:' + mx.toFixed(3) + ' mm';
          }
        }
      }
    },
    scales: {
      x: { ticks:{color:CTICK, maxRotation:45, autoSkip:true, maxTicksLimit:20}, grid:{color:CGRID} },
      y: { ticks:{color:CTICK}, grid:{color:CGRID}, title:{display:true,text:'Precision (mm)',color:CTICK}, min:0 }
    }
  }
});

// ==================================================================
// BRUSH STATE + HELPERS -- declared before Chart.register to avoid
// temporal dead zone crash when top chart fires afterDraw on creation
// ==================================================================
var brushS = null, brushE = null, dragging = false;

// px -> cycle using chart area ratio (scale-type agnostic)
function pixToCycle(chart, px) {
  const ca = chart.chartArea;
  const ratio = Math.max(0, Math.min(1, (px - ca.left) / (ca.right - ca.left)));
  return Math.max(1, Math.min(MAX_CYCLE, Math.round(1 + ratio * (MAX_CYCLE - 1))));
}

// cycle -> px using same ratio math
function cycleToPx(chart, cycle) {
  const ca = chart.chartArea;
  return ca.left + (cycle - 1) / (MAX_CYCLE - 1) * (ca.right - ca.left);
}

// Brush plugin: draws selection rectangle on sparkline ONLY
const brushPlugin = {
  id: 'brushDraw',
  afterDraw(chart) {
    if (chart.canvas.id !== 'chart-spark') return;
    if (brushS === null) return;
    const {ctx, chartArea: ca} = chart;
    const x1 = cycleToPx(chart, Math.min(brushS, brushE));
    const x2 = cycleToPx(chart, Math.max(brushS, brushE));
    ctx.save();
    ctx.fillStyle   = 'rgba(137,180,250,0.2)';
    ctx.strokeStyle = 'rgba(137,180,250,0.8)';
    ctx.lineWidth   = 1;
    ctx.fillRect  (x1, ca.top, x2 - x1, ca.height);
    ctx.strokeRect(x1, ca.top, x2 - x1, ca.height);
    ctx.restore();
  }
};
Chart.register(brushPlugin);

// ==================================================================
// MIDDLE CHART: full sparkline (all cycles, no points for perf)
// ==================================================================
const allLabels = Array.from({length:MAX_CYCLE}, (_,i) => i+1);

const chartSpark = new Chart(document.getElementById('chart-spark'), {
  type: 'line',
  data: {
    labels: allLabels,
    datasets: [
      { label:'Up',   data:UP_VALS,   borderColor:CUP,   borderWidth:1, pointRadius:0, tension:0, spanGaps:false },
      { label:'Down', data:DOWN_VALS, borderColor:CDOWN, borderWidth:1, pointRadius:0, tension:0, spanGaps:false },
    ]
  },
  options: {
    animation:false, responsive:true,
    plugins: { legend:{display:false}, tooltip:{enabled:false} },
    scales: { x:{display:false}, y:{display:false, min:0} }
  }
});

const sparkCanvas = document.getElementById('chart-spark');
sparkCanvas.addEventListener('mousedown', function(e) {
  dragging = true;
  brushS = brushE = pixToCycle(chartSpark, e.offsetX);
  chartSpark.update('none');
});
sparkCanvas.addEventListener('mousemove', function(e) {
  if (!dragging) return;
  brushE = pixToCycle(chartSpark, e.offsetX);
  chartSpark.update('none');
  refreshDetail();
});
sparkCanvas.addEventListener('mouseup', function(e) {
  if (!dragging) return;
  dragging = false;
  brushE = pixToCycle(chartSpark, e.offsetX);
  if (Math.abs(brushE - brushS) < 2) { brushS = null; brushE = null; }
  chartSpark.update('none');
  refreshDetail();
});
sparkCanvas.addEventListener('mouseleave', function() {
  if (dragging) { dragging = false; chartSpark.update('none'); refreshDetail(); }
});
// Double-click clears the brush
sparkCanvas.addEventListener('dblclick', function() {
  brushS = null; brushE = null;
  chartSpark.update('none');
  refreshDetail();
});

// ==================================================================
// BOTTOM CHART: detail view (updates on brush change)
// ==================================================================
const chartDetail = new Chart(document.getElementById('chart-detail'), {
  type: 'line',
  data: {
    labels: [],
    datasets: [
      {
        label: 'Up (mm)',
        data: [], borderColor: CUP, borderWidth: 1.5,
        pointRadius: [], pointBackgroundColor: [],
        tension: 0.2, spanGaps: false
      },
      {
        label: 'Down (mm)',
        data: [], borderColor: CDOWN, borderWidth: 1.5,
        pointRadius: [], pointBackgroundColor: [],
        tension: 0.2, spanGaps: false
      }
    ]
  },
  options: {
    animation: false, responsive: true,
    plugins: {
      legend: { labels:{color:CTICK} },
      tooltip: {
        callbacks: {
          label: function(ctx) {
            if (ctx.parsed.y === null) return ctx.dataset.label.split(' ')[0] + ': not detected';
            return ctx.dataset.label.split(' ')[0] + ': ' + ctx.parsed.y.toFixed(3) + ' mm';
          }
        }
      }
    },
    scales: {
      x: { ticks:{color:CTICK, maxRotation:45, autoSkip:true, maxTicksLimit:30}, grid:{color:CGRID} },
      y: { ticks:{color:CTICK}, grid:{color:CGRID}, title:{display:true,text:'Precision (mm)',color:CTICK}, min:0 }
    }
  }
});

function refreshDetail() {
  const s = (brushS !== null) ? Math.min(brushS,brushE) : 1;
  const e = (brushS !== null) ? Math.max(brushS,brushE) : MAX_CYCLE;
  const count = e - s + 1;
  // show points only if the window is small enough to be readable
  const showPts = count <= 300;
  const ptRad   = count <= 100 ? 4 : 2;

  const labels = [], uData = [], dData = [], uCol = [], dCol = [];
  for (let c = s; c <= e; c++) {
    const u = UP_VALS[c-1], d = DOWN_VALS[c-1];
    labels.push(c);
    uData.push(u); uCol.push(ptColor(u));
    dData.push(d); dCol.push(ptColor(d));
  }

  const ds = chartDetail.data.datasets;
  chartDetail.data.labels  = labels;
  ds[0].data               = uData;
  ds[0].pointRadius        = showPts ? uData.map(() => ptRad) : 0;
  ds[0].pointBackgroundColor = uCol;
  ds[1].data               = dData;
  ds[1].pointRadius        = showPts ? dData.map(() => ptRad) : 0;
  ds[1].pointBackgroundColor = dCol;
  chartDetail.update('none');

  const titleEl = document.getElementById('detail-title');
  const infoEl  = document.getElementById('brush-info');
  if (brushS !== null) {
    titleEl.innerHTML = '&#9632; DETAIL VIEW &mdash; cycles ' + s + ' to ' + e + ' (' + count + ' cycles)';
    infoEl.textContent = 'Showing cycles ' + s + ' \u2013 ' + e + '  |  double-click sparkline to reset';
  } else {
    titleEl.innerHTML = '&#9632; DETAIL VIEW &mdash; all cycles';
    infoEl.textContent = 'Drag on the sparkline to zoom into a range in the detail view below';
  }
}

// Init with all data
refreshDetail();
</script>
</body></html>
)ENDHTML";

    std::cout << "[chart] saved: " << path << "\n";
}

// ===========================================================================
// One capture + precision measurement (unchanged logic, takes position label)
// ===========================================================================
static Record measure(XCamera* cam, const Config& cfg,
    const BoardConfig& board, int W, int H,
    int capture_num, int cycle, const std::string& position)
{
    Record rec;
    rec.timestamp = nowStr();
    rec.cycle = cycle;
    rec.position = position;
    rec.detected = false;
    rec.precision_mm = 0;
    rec.valid_points = 0;

    char ts[30] = "";
    int ret = cam->captureData(capture_num, ts);
    if (ret != 0) {
        std::cout << "[" << rec.timestamp << "] [" << position << "] capture failed (" << ret << ")\n";
        return rec;
    }

    size_t bsz = (size_t)W * H, psz = (size_t)W * H * 3;
    unsigned char* bright = (unsigned char*)malloc(bsz);
    float* pc = (float*)malloc(psz * sizeof(float));
    memset(bright, 0, bsz);
    memset(pc, 0, psz * sizeof(float));
    cam->getBrightnessData(bright);
    cam->getPointcloudData(pc);

    cv::Mat img(H, W, CV_8UC1, bright);
    cv::Mat inv; cv::bitwise_not(img, inv);

    cv::Size bsz2(board.cols, board.rows);
    std::vector<cv::Point2f> cpts;
    bool found = cv::findCirclesGrid(inv, bsz2, cpts,
        cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);

    if (!found) {
        std::cout << "[" << rec.timestamp << "] [" << position << "] board not detected\n";
        free(bright); free(pc);
        return rec;
    }

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
        std::cout << "[" << rec.timestamp << "] [" << position << "] too few 3D points ("
            << measured.size() << ")\n";
        return rec;
    }

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
    std::cout << "[" << rec.timestamp << "] [cycle " << cycle << "] [" << position << "] "
        << std::fixed << std::setprecision(3) << rec.precision_mm
        << " mm  pts=" << N << "  [" << tag << "]\n";
    return rec;
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
    if (argc >= 4) cfg.com_port = argv[3];
    if (argc >= 5) cfg.stay_time_sec = std::atoi(argv[4]);

    if (cfg.ip.empty()) {
        std::cout << "ERROR: no camera IP in camera_config.json or argv[1]\n"; return 1;
    }
    if (cfg.com_port.empty()) {
        std::cout << "ERROR: no serial port in camera_config.json (servo.com_port) or argv[3]\n"; return 1;
    }

    BoardConfig board = getBoard(cfg.calibration_board);

    // Output files
    std::string date_tag = dateStr();
    std::string csv_path = "precision_log_" + date_tag + ".csv";
    std::string chart_path = "precision_chart_" + date_tag + ".html";

    // ------------------------------------------------------------------
    // Open serial port
    // ------------------------------------------------------------------
    HANDLE hSerial = openSerial(cfg.com_port, cfg.baud_rate);
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cout << "ERROR: cannot open serial port " << cfg.com_port << "\n"; return 1;
    }
    std::cout << "[serial] " << cfg.com_port << " @ " << cfg.baud_rate << " baud opened\n";

    // Pre-build framed commands
    auto up_frame = modbusFrame(RAW_UP, sizeof(RAW_UP));
    auto down_frame = modbusFrame(RAW_DOWN, sizeof(RAW_DOWN));

    // Print frames for verification
    std::cout << "[serial] UP   frame: ";
    for (auto b : up_frame)   std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    std::cout << std::dec << "\n";
    std::cout << "[serial] DOWN frame: ";
    for (auto b : down_frame) std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    std::cout << std::dec << "\n";

    // ------------------------------------------------------------------
    // Connect camera
    // ------------------------------------------------------------------
    XCamera* cam = (XCamera*)createXCamera();
    std::cout << "connecting to " << cfg.ip << " ...\n";
    int ret = cam->connect(cfg.ip.c_str());
    if (ret != 0) {
        std::cout << "connect failed (" << ret << ")\n";
        destroyXCamera(cam); CloseHandle(hSerial); return 1;
    }
    std::cout << "connected\n";

    int W = 0, H = 0;
    cam->getCameraResolution(&W, &H);
    std::cout << "resolution " << W << "x" << H << "\n\n";

    cam->setParamLedCurrent(cfg.led_current);
    cam->setParamCameraExposure((float)cfg.camera_exposure_time);
    cam->setParamCameraGain((double)cfg.camera_gain);
    cam->setParamCameraConfidence((float)cfg.confidence);

    XemaEngine eng = (cfg.engine == 2) ? XemaEngine::Black :
        (cfg.engine == 1) ? XemaEngine::Reflect : XemaEngine::Normal;
    cam->setCaptureEngine(eng);

    int capture_num = 1;
    if (cfg.exposure_model == 1) {
        capture_num = cfg.mixed_exposure_num;
        cam->setParamMultipleExposureModel(1);
        cam->setParamMixedHdr(capture_num,
            cfg.mixed_exposure_param_list,
            cfg.mixed_led_param_list);
    }
    else if (cfg.exposure_model == 2) {
        capture_num = cfg.repetition_count;
        cam->setParamMultipleExposureModel(2);
        cam->setParamRepetitionExposureNum(capture_num);
    }

    std::cout << "=== Servo Precision Test ===\n"
        << "  stay time  : " << cfg.stay_time_sec << "s per position\n"
        << "  csv        : " << csv_path << "\n"
        << "  chart      : " << chart_path << "\n"
        << "  Press Ctrl+C to stop and export final chart.\n\n";

    // ------------------------------------------------------------------
    // Main loop: UP -> measure -> DOWN -> measure -> repeat
    // ------------------------------------------------------------------
    std::vector<Record> records;
    int cycle = 0;

    while (g_running) {
        cycle++;
        std::cout << "\n--- Cycle " << cycle << " ---\n";

        // ---- UP ----
        if (!sendSerial(hSerial, up_frame)) {
            std::cout << "[serial] WARNING: UP command write failed\n";
        }
        else {
            std::cout << "[serial] UP command sent (camera moves DOWN)\n";
        }
        std::cout << "[wait] staying " << cfg.stay_time_sec << "s for motion to complete...\n";
        // Interruptible sleep
        for (int i = 0; i < cfg.stay_time_sec * 10 && g_running; i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (!g_running) break;

        Record up_rec = measure(cam, cfg, board, W, H, capture_num, cycle, "down");
        records.push_back(up_rec);
        writeCSV(csv_path, records);

        if (!g_running) break;

        // ---- DOWN ----
        if (!sendSerial(hSerial, down_frame)) {
            std::cout << "[serial] WARNING: DOWN command write failed\n";
        }
        else {
            std::cout << "[serial] DOWN command sent (camera moves UP)\n";
        }
        std::cout << "[wait] staying " << cfg.stay_time_sec << "s for motion to complete...\n";
        for (int i = 0; i < cfg.stay_time_sec * 10 && g_running; i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (!g_running) break;

        Record dn_rec = measure(cam, cfg, board, W, H, capture_num, cycle, "up");
        records.push_back(dn_rec);
        writeCSV(csv_path, records);

        // Chart after every cycle
        writeChart(chart_path, records);
    }

    // ------------------------------------------------------------------
    // Final export
    // ------------------------------------------------------------------
    std::cout << "\n[exit] stopping...\n";
    writeCSV(csv_path, records);
    writeChart(chart_path, records);

    cam->disconnect(cfg.ip.c_str());
    destroyXCamera(cam);
    CloseHandle(hSerial);

    std::cout << "[exit] done.  " << records.size() << " measurements (" << cycle << " cycles).\n";
    return 0;
}