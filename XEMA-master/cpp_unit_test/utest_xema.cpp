#include <vector>
#include <string>
#include <array>
#include <fstream>
#include <filesystem>
#include <cstdlib>

#include "gtest/gtest.h"
#include "easylogging++.h"
#include "json.hpp"
#include "xcamera.h"
#include "enumerate.h"

using namespace XEMA;

INITIALIZE_EASYLOGGINGPP

namespace {

struct TestConfig {
    int repeat_count = 1;
    std::string log_path = "./logs";
    std::string camera_ip = "";
    int confidence = 10;
    float gain = 0.0F;
    int smoothing = 1;
    int led_current = 1023;
    float exposure = 30000.0F;
    float brightness_exposure = 36000.0F;
    float brightness_gain = 10.0F;
    int brightness_exposure_model = 1;
    int multi_exposure_num = 2;
    std::array<int, 6> multi_led_params = {100, 1023, 1023, 1023, 1023, 1023};
    std::array<int, 6> multi_exposure_params = {6000, 30000, 60000, 60000, 60000, 60000};
};

TestConfig g_config;
XCamera* g_camera = nullptr;
std::string g_selected_ip;
int g_width = 0;
int g_height = 0;
int g_channels = 1;
bool g_test_failed = false;
std::string g_log_dir = "./logs";

std::string JoinPath(const std::string& directory, const std::string& file_name)
{
    return (std::filesystem::path(directory) / file_name).string();
}

bool EnsureLogDirectory(const std::string& log_dir)
{
    if (log_dir.empty()) {
        return false;
    }

    std::error_code error_code;
    std::filesystem::create_directories(log_dir, error_code);
    if (error_code) {
        return false;
    }

    return true;
}

void InitializeProcessLog(const std::string& log_dir)
{
    if (!EnsureLogDirectory(log_dir)) {
        return;
    }

    const std::string log_file_path = JoinPath(log_dir, "utest_xema.txt");
    el::Configurations cfg;
    cfg.setToDefault();
    cfg.setGlobally(el::ConfigurationType::ToFile, "true");
    cfg.setGlobally(el::ConfigurationType::Filename, log_file_path);
    cfg.setGlobally(el::ConfigurationType::ToStandardOutput, "true");
    cfg.setGlobally(el::ConfigurationType::Format, "%msg");
    el::Loggers::reconfigureAllLoggers(cfg);
    g_log_dir = log_dir;
}

void LoadConfig(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        LOG(INFO) << "[Config] " << path << " not found, using defaults.";
        return;
    }

    try {
        nlohmann::json j;
        file >> j;

        auto load_int = [](const nlohmann::json& obj, const char* key, int& out) {
            if (obj.contains(key)) {
                out = obj[key].get<int>();
            }
        };
        auto load_float = [](const nlohmann::json& obj, const char* key, float& out) {
            if (obj.contains(key)) {
                out = obj[key].get<float>();
            }
        };
        auto load_string = [](const nlohmann::json& obj, const char* key, std::string& out) {
            if (obj.contains(key)) {
                out = obj[key].get<std::string>();
            }
        };
        auto load_int_array6 = [](const nlohmann::json& obj, const char* key, std::array<int, 6>& out) {
            if (!obj.contains(key) || !obj[key].is_array()) {
                return;
            }
            const auto& src = obj[key];
            const size_t count = (src.size() < out.size()) ? src.size() : out.size();
            for (size_t i = 0; i < count; ++i) {
                out[i] = src[i].get<int>();
            }
        };

        load_int(j, "repeat_count", g_config.repeat_count);
        load_string(j, "log_path", g_config.log_path);
        load_string(j, "camera_ip", g_config.camera_ip);
        load_int(j, "confidence", g_config.confidence);
        load_float(j, "gain", g_config.gain);
        load_int(j, "smoothing", g_config.smoothing);
        load_int(j, "led_current", g_config.led_current);
        load_float(j, "exposure", g_config.exposure);
        load_float(j, "brightness_exposure", g_config.brightness_exposure);
        load_float(j, "brightness_gain", g_config.brightness_gain);
        load_int(j, "brightness_exposure_model", g_config.brightness_exposure_model);
        load_int(j, "multi_exposure_num", g_config.multi_exposure_num);
        load_int_array6(j, "multi_led_params", g_config.multi_led_params);
        load_int_array6(j, "multi_exposure_params", g_config.multi_exposure_params);
    } catch (const std::exception& e) {
        LOG(ERROR) << "[Config] Parse error: " << e.what() << ", using defaults.";
    }

    LOG(INFO) << "[Config] Loaded from " << path;
}

void FetchAndValidateResultData()
{
    std::vector<float> depth_data(static_cast<size_t>(g_width) * static_cast<size_t>(g_height));
    std::vector<float> height_map_data(static_cast<size_t>(g_width) * static_cast<size_t>(g_height));
    std::vector<float> point_cloud_data(static_cast<size_t>(g_width) * static_cast<size_t>(g_height) * 3U);
    ASSERT_EQ(g_camera->getDepthData(depth_data.data()), 0);
    ASSERT_EQ(g_camera->getHeightMapData(height_map_data.data()), 0);
    ASSERT_EQ(g_camera->getPointcloudData(point_cloud_data.data()), 0);

    if (g_channels == 1) {
        std::vector<unsigned char> brightness_data(static_cast<size_t>(g_width) * static_cast<size_t>(g_height));
        ASSERT_EQ(g_camera->getBrightnessData(brightness_data.data()), 0);
    } else {
        std::vector<unsigned char> color_brightness_data(static_cast<size_t>(g_width) * static_cast<size_t>(g_height) * 3U);
        ASSERT_EQ(g_camera->getColorBrightnessData(color_brightness_data.data(), XemaColor::Rgb), 0);
    }

    float plane_r[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};
    float plane_t[3] = {0.0F, 0.0F, 0.0F};
    ASSERT_EQ(g_camera->getHeightMapDataBaseParam(plane_r, plane_t, height_map_data.data()), 0);
}

bool InitCamera()
{
    int camera_num = 0;
    if (DfUpdateDeviceList(0, camera_num) != 0 || camera_num <= 0) {
        return false;
    }

    std::vector<DeviceBaseInfo> base_infos(static_cast<size_t>(camera_num));
    int info_size = static_cast<int>(sizeof(DeviceBaseInfo) * base_infos.size());
    if (DfGetAllDeviceBaseInfo(base_infos.data(), &info_size) != 0) {
        return false;
    }

    if (!g_config.camera_ip.empty()) {
        for (const auto& info : base_infos) {
            if (g_config.camera_ip == info.ip) {
                g_selected_ip = info.ip;
                break;
            }
        }
        if (g_selected_ip.empty()) {
            return false;
        }
    } else {
        g_selected_ip = base_infos[0].ip;
    }

    g_camera = static_cast<XCamera*>(createXCamera());
    if (g_camera == nullptr) {
        return false;
    }

    if (g_camera->connect(g_selected_ip.c_str()) != 0) {
        destroyXCamera(g_camera);
        g_camera = nullptr;
        return false;
    }

    if (g_camera->getCameraResolution(&g_width, &g_height) != 0 || g_width <= 0 || g_height <= 0) {
        return false;
    }

    if (g_camera->getCameraChannels(&g_channels) != 0 || (g_channels != 1 && g_channels != 3)) {
        return false;
    }

    return true;
}

void CleanupCamera()
{
    if (g_camera != nullptr) {
        g_camera->disconnect(g_selected_ip.c_str());
        destroyXCamera(g_camera);
        g_camera = nullptr;

        const std::string firmware_log_path = JoinPath(g_log_dir, "firmware_xema_log.log");
        const std::string remote_log = "dexforce@" + g_selected_ip + ":/opt/xema/xema_log.log";
        std::string cmd;
#if defined(_WIN32)
        const std::string trust_cmd = "echo y | plink -ssh -pw dexforce dexforce@" + g_selected_ip + " \"exit\"";
        const int trust_ret = system(trust_cmd.c_str());
        if (trust_ret != 0)
        {
            LOG(WARNING) << "Host key pre-accept failed or skipped (return code: " << trust_ret << ")";
        }
        cmd = "pscp -pw dexforce -batch " + remote_log + " \"" + firmware_log_path + "\"";
#elif defined(__linux__)
        cmd = "sshpass -p dexforce scp -o StrictHostKeyChecking=no " + remote_log + " " + firmware_log_path;
#endif
        const int log_ret = system(cmd.c_str());
        if (log_ret == 0)
        {
            LOG(INFO) << "Firmware log retrieved successfully.";
        }
        else
        {
            LOG(ERROR) << "Failed to retrieve firmware log (return code: " << log_ret << ")";
        }
    }
    g_selected_ip.clear();
    g_width = 0;
    g_height = 0;
    g_channels = 1;
}

class XemaCameraTest : public ::testing::Test {
};

class FailureDetectorListener : public ::testing::EmptyTestEventListener {
public:
    void OnTestEnd(const ::testing::TestInfo& test_info) override
    {
        if (test_info.result()->Failed()) {
            g_test_failed = true;
        }
    }
};

TEST_F(XemaCameraTest, QueryDeviceInfo)
{
    ASSERT_NE(g_camera, nullptr);

    // char sdk_version[64] = {0};
    // ASSERT_EQ(g_camera->getSdkVersion(sdk_version), 0);
    // EXPECT_NE(std::string(sdk_version), "");

    // char firmware_version[64] = {0};
    // ASSERT_EQ(g_camera->getFirmwareVersion(firmware_version), 0);
    // EXPECT_NE(std::string(firmware_version), "");

    // std::string product_info;
    // ASSERT_EQ(g_camera->getProductInfo(&product_info), 0);
    // EXPECT_FALSE(product_info.empty());

    // double network_speed = 0.0;
    // ASSERT_EQ(g_camera->testNetworkSpeed(&network_speed), 0);
    // EXPECT_GE(network_speed, 0.0);

    CalibrationParam calib_param;
    ASSERT_EQ(g_camera->getCalibrationParam(&calib_param), 0);
}

TEST_F(XemaCameraTest, CaptureSingleExposure)
{
    ASSERT_NE(g_camera, nullptr);

    ASSERT_EQ(g_camera->setParamCameraConfidence(static_cast<float>(g_config.confidence)), 0);
    ASSERT_EQ(g_camera->setParamCameraGain(g_config.gain), 0);
    ASSERT_EQ(g_camera->setParamSmoothing(g_config.smoothing), 0);
    ASSERT_EQ(g_camera->setParamGenerateBrightness(1, g_config.brightness_exposure), 0);
    ASSERT_EQ(g_camera->setParamBrightnessGain(g_config.brightness_gain), 0);
    ASSERT_EQ(g_camera->setParamBrightnessExposureModel(g_config.brightness_exposure_model), 0);
    ASSERT_EQ(g_camera->setCaptureEngine(XemaEngine::Black), 0);

    ASSERT_EQ(g_camera->setParamLedCurrent(g_config.led_current), 0);
    ASSERT_EQ(g_camera->setParamCameraExposure(g_config.exposure), 0);

    char timestamp_data[64] = {0};
    ASSERT_EQ(g_camera->captureData(1, timestamp_data), 0);

    FetchAndValidateResultData();
}

TEST_F(XemaCameraTest, CaptureMultiExposure)
{
    ASSERT_NE(g_camera, nullptr);

    ASSERT_EQ(g_camera->setParamCameraConfidence(static_cast<float>(g_config.confidence)), 0);
    ASSERT_EQ(g_camera->setParamCameraGain(g_config.gain), 0);
    ASSERT_EQ(g_camera->setParamSmoothing(g_config.smoothing), 0);
    ASSERT_EQ(g_camera->setParamGenerateBrightness(1, g_config.brightness_exposure), 0);
    ASSERT_EQ(g_camera->setParamBrightnessGain(g_config.brightness_gain), 0);
    ASSERT_EQ(g_camera->setParamBrightnessExposureModel(g_config.brightness_exposure_model), 0);
    ASSERT_EQ(g_camera->setCaptureEngine(XemaEngine::Black), 0);

    ASSERT_GE(g_config.multi_exposure_num, 2);
    ASSERT_LE(g_config.multi_exposure_num, 6);

    int exposure_param[6] = {
        g_config.multi_exposure_params[0], g_config.multi_exposure_params[1], g_config.multi_exposure_params[2],
        g_config.multi_exposure_params[3], g_config.multi_exposure_params[4], g_config.multi_exposure_params[5]};
    int led_param[6] = {
        g_config.multi_led_params[0], g_config.multi_led_params[1], g_config.multi_led_params[2],
        g_config.multi_led_params[3], g_config.multi_led_params[4], g_config.multi_led_params[5]};

    ASSERT_EQ(g_camera->setParamMixedHdr(g_config.multi_exposure_num, exposure_param, led_param), 0);
    ASSERT_EQ(g_camera->setParamMultipleExposureModel(1), 0);

    char timestamp_data[64] = {0};
    ASSERT_EQ(g_camera->captureData(g_config.multi_exposure_num, timestamp_data), 0);

    FetchAndValidateResultData();
}

}  // namespace

int main(int argc, char** argv)
{
    std::string config_path = "config.json";
    std::string log_path_override;
    int repeat_count_override = -1;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg.compare(0, 11, "--log_path=") == 0)
        {
            log_path_override = arg.substr(11);
            continue;
        }

        const std::string repeat_prefix = "--repeat_count=";
        if (arg.compare(0, repeat_prefix.size(), repeat_prefix) == 0)
        {
            try
            {
                repeat_count_override = std::stoi(arg.substr(repeat_prefix.size()));
            }
            catch (const std::exception&)
            {
                std::cerr << "Invalid --repeat_count option." << std::endl;
                return 1;
            }
            continue;
        }

        if (arg.rfind("--gtest_", 0) == 0)
        {
            continue;
        }

        if (arg.rfind("--", 0) == 0)
        {
            std::cerr << "Unknown option: " << arg << std::endl;
            return 1;
        }

        config_path = arg;
    }

    LoadConfig(config_path);
    if (!log_path_override.empty()) {
        g_config.log_path = log_path_override;
    }
    InitializeProcessLog(g_config.log_path);

    if (repeat_count_override > 0) {
        g_config.repeat_count = repeat_count_override;
    }
    if (g_config.repeat_count <= 0) {
        LOG(ERROR) << "repeat_count must be greater than 0.";
        return 1;
    }

    ::testing::InitGoogleTest(&argc, argv);
    ::testing::TestEventListeners& listeners = ::testing::UnitTest::GetInstance()->listeners();
    listeners.Append(new FailureDetectorListener());

    if (!InitCamera()) {
        LOG(INFO) << "No available camera or camera initialization failed.";
        return 0;
    }

    int test_result = 0;
    for (int repeat = 0; repeat < g_config.repeat_count; ++repeat)
    {
        LOG(INFO) << "[Repeat " << (repeat + 1) << "/" << g_config.repeat_count << "]";
        g_test_failed = false;
        test_result = RUN_ALL_TESTS();
        if (g_test_failed) {
            LOG(ERROR) << "[Stop] Test failed in repeat " << (repeat + 1) << ".";
            break;
        }
    }

    CleanupCamera();
    return test_result;
}
