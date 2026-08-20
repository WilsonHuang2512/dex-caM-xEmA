#pragma once

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QTextEdit>
#include <QImage>
#include <QStringList>
#include <QRadioButton>
#include <QButtonGroup>
#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <opencv2/core.hpp>

// Confirmed real API from XEMA-master/sdk/open_cam3d.h, cross-checked against BOTH
// XEMA-master/cmd/cmd.cpp's get_brightness() AND the real vendor GUI
// (XEMA-master/gui/camera_capture_gui.cpp)'s continuous-capture path
// (do_timeout_capture_slot -> captureOneFrameBaseThread):
//   DfConnect(ip) / DfDisconnect(ip)
//   DfGetCameraResolution(&w, &h)
//   DfGetFirmwareVersion(char[64])
//   DfSetParamLedCurrent(int) / DfGetParamLedCurrent(int&)
//   DfCaptureData(exposure_num, timestamp) -- blocking, triggers the REAL structured-light
//     capture (projector fires the actual fringe/pattern sequence, firmware reconstructs
//     depth + brightness from it)
//   DfGetBrightnessData(unsigned char*) -- pulls the brightness result out of that capture
//
// IMPORTANT: this GUI used to call DfCaptureBrightnessData(buffer, XemaColor::Gray) instead,
// which is a DIFFERENT, lightweight standalone grab that skips pattern projection and
// reconstruction entirely -- that's why captures looked fast and didn't show the projector's
// pattern the way the real vendor GUI's captures do. DfCaptureData + DfGetBrightnessData is
// the actual path captureOneFrameBaseThread uses, confirmed from its source.
//
// IMPORTANT: exposure/gain for THIS capture path (DfCaptureData + DfGetBrightnessData) are
// DfSetParamCameraExposure(float) and DfSetParamCameraGain(float) -- confirmed from the real
// vendor GUI's do_spin_camera_exposure_changed / do_doubleSpin_gain handlers, which call
// exactly these two. DfSetParamGenerateBrightness/DfSetParamBrightnessGain are a SEPARATE,
// independent brightness-image feature (model 1/2/3) that only affects
// DfCaptureBrightnessData's output -- a different capture call this GUI no longer uses.
// Using the wrong pair here was an earlier bug: it looked like "successfully" setting a
// param (SUCCESS return code, correct readback) while having zero effect on what Capture
// actually showed, because it was tuning a parameter DfCaptureData never reads.
// CORRECTED (see .cpp captureLoopThreadFunc comment): DfGetBrightnessData -- the actual
// open_cam3d.cpp implementation this build links -- ALREADY does pixel-type-aware
// Bayer->RGB->Gray conversion internally when pixel_type_ is BayerRG8. It never returns raw
// undemosaiced data. pixel_type_ is still tracked/logged here (useful diagnostic info, shown
// in the connect log), but no longer drives a separate capture code path.
#include "../sdk/open_cam3d.h"
#include "../sdk/xema_enums.h"

// Capture engine (from XEMA-master/sdk/open_cam3d.h's XemaEngine enum: Normal=0, Reflect=1,
// Black=2). DfCaptureData branches internally on this -- Normal uses DfGetFrame04, Reflect
// uses DfGetFrame06, Black uses DfGetFrame06Mono12 -- genuinely different reconstruction
// paths, not just a label. The SDK's raw default (before any DfSetCaptureEngine call) is
// Reflect, confirmed from open_cam3d.cpp's global `XemaEngine engine_ = XemaEngine::Reflect;`.
// The real vendor GUI instead explicitly sets this from its own last-saved setting right
// after connecting (do_comboBox_activated_engine), so if that saved setting isn't Reflect,
// the real GUI and this tool would silently be running different capture pipelines even with
// identical exposure/gain/LED -- a very plausible explanation for board recognition working
// in one and not the other.

// Board spacing options -- same set used in the earlier scan tool. This value is purely
// informational for board detection here (it doesn't change what OpenCV looks for -- the
// grid topology below is what actually drives detection); it's tracked/displayed/persisted
// so the person doing the recognition knows which physical board they're pointed at.
enum class XemaBoardSpacingMm
{
	Spacing4 = 4,
	Spacing12 = 12,
	Spacing20 = 20,
	Spacing40 = 40,
	Spacing80 = 80,
};

// Minimal XEMA camera control GUI: connect, adjust LED/gain/exposure, continuously capture
// grayscale frames with overexposure highlighting and calibration-board recognition,
// disconnect. No calibration math, no disk-saving workflow.
class XemaCameraWindow : public QWidget
{
	Q_OBJECT

public:
	XemaCameraWindow(QWidget* parent = nullptr);
	~XemaCameraWindow();

signals:
	void connectFinished(bool ok, QString message);
	void captureFinished(bool ok, QString message, QImage image);
	void applyParamsFinished(QString message);
	void disconnectFinished(QString message);

private slots:
	void onConnectClicked();
	void onDisconnectClicked();
	void onApplyParamsClicked();
	void onCaptureToggled();
	void onConnectFinished(bool ok, QString message);
	void onCaptureFinished(bool ok, QString message, QImage image);
	void onApplyParamsFinished(QString message);
	void onDisconnectFinished(QString message);
	void onBoardSpacingChanged();
	void onCaptureEngineChanged(int index);
	void onSaveFrameClicked();

private:
	void log(const QString& msg);
	void setConnectedUiState(bool connected);
	void connectThreadFunc(QString ip);
	void applyExposureRangeForProjector(); // sets spin_exposure_'s range based on projector_version_ -- see .cpp for the real 3010/4710 bounds, confirmed from camera_capture_gui.cpp's setCameraConfigParam()
	void logCurrentParamsInto(const QString& context, QStringList& lines_out); // reads LED/exposure/gain straight from the camera and appends log lines to lines_out -- doesn't touch the GUI directly, so it's safe to call from a background thread

	// Runs continuously on a background thread while capturing_ is true: blocking-captures
	// one grayscale frame, builds the overexposure-highlighted overlay, emits
	// captureFinished, then immediately captures the next one. No fixed-interval timer --
	// each capture is blocking over the network already, so back-to-back calls naturally
	// throttle to the camera's real capture speed. Stops itself once capturing_ goes false
	// (checked between frames, so it exits after the in-flight capture finishes, not mid-call).
	// Sets capture_thread_active_ true on entry, false just before returning -- that's how
	// applyParamsThreadFunc knows the in-flight capture has actually finished, not just that
	// a stop was requested.
	void captureLoopThreadFunc();

	// Runs entirely on a background thread, mirroring the real vendor GUI's pattern (every
	// param-change handler in camera_capture_gui.cpp does stopCapturingOneFrameBaseThread ->
	// set param -> do_pushButton_capture_continuous to resume). Doing this on the GUI thread
	// was the bug: DfCaptureData holds the SDK's internal command mutex for the whole
	// capture, and DfSetParamCameraExposure/DfSetParamCameraGain block on that same mutex --
	// called synchronously from onApplyParamsClicked, that froze the whole window until the
	// in-flight capture finished. This function stops the loop, WAITS for
	// capture_thread_active_ to actually go false (not just capturing_ requested-false),
	// applies the params, then restarts the loop if it was running -- all off the GUI thread.
	void applyParamsThreadFunc(int led, float gain, float exposure);

	// Same reasoning as applyParamsThreadFunc: DfDisconnect can block on the SDK's internal
	// command mutex if a capture is still in flight, so this waits for capture_thread_active_
	// to clear (off the GUI thread) before disconnecting.
	void disconnectThreadFunc(QString ip);

	// Converts a grayscale frame to a BGR overlay with overexposed pixels (>= threshold,
	// default near-saturation) marked red, and returns what fraction of pixels were flagged.
	double markOverexposure(const cv::Mat& gray, cv::Mat& overlay_bgr_out);

	// Asymmetric circle-grid board detection, drawn onto the same overlay markOverexposure
	// produced (green dots + connecting grid if found). Pure OpenCV, no GUI touches -- safe
	// to call from captureLoopThreadFunc's background thread.
	// ASSUMPTION (not verified against real board hardware for every spacing): grid topology
	// is a constant 9 cols x 13 rows regardless of which spacing (4/12/20/40/80mm) is
	// selected -- only the physical dot spacing differs between boards, not the row/col
	// count. This is the same assumption the earlier scan tool made, based on
	// ScanToolWindow.cpp hardcoding 13x9 specifically for the confirmed 80mm/model-2002
	// variant. If a different spacing's physical board actually uses a different row/col
	// count, detection on that spacing will silently fail to find a real board rather than
	// report a wrong result -- it's a false negative risk, not a false positive one.
	bool detectBoard(const cv::Mat& gray, std::vector<cv::Point2f>& points_out);
	cv::Size boardGridSize() const; // (cols, rows) -- see detectBoard's ASSUMPTION comment above

	QImage matToQImage(const cv::Mat& mat_bgr);
	void loadConfig();
	void saveConfig();

	static const int kOverexposureThreshold = 250; // 0-255 grayscale value at/above which a pixel counts as overexposed
	static constexpr double kOverexposureWarnPercent = 1.0; // log a warning once flagged pixels exceed this % of the frame
	static const int kCaptureStopTimeoutMs = 8000; // how long applyParamsThreadFunc waits for an in-flight capture to finish before giving up and applying anyway

	// UI
	QLineEdit* edit_ip_;
	QPushButton* btn_connect_;
	QPushButton* btn_disconnect_;
	QLabel* label_firmware_;

	QSpinBox* spin_led_;
	QDoubleSpinBox* spin_gain_;
	QDoubleSpinBox* spin_exposure_;
	QPushButton* btn_apply_params_;

	QPushButton* btn_capture_; // toggles continuous capture on/off
	QPushButton* btn_save_frame_; // saves the raw, unprocessed last captured frame to disk for inspection
	QLabel* label_image_;
	QLabel* label_overexposure_;
	QLabel* label_board_;
	QTextEdit* log_view_;

	QButtonGroup* group_board_spacing_;
	QRadioButton* radio_spacing_4_;
	QRadioButton* radio_spacing_12_;
	QRadioButton* radio_spacing_20_;
	QRadioButton* radio_spacing_40_;
	QRadioButton* radio_spacing_80_;
	XemaBoardSpacingMm board_spacing_ = XemaBoardSpacingMm::Spacing80;

	QComboBox* combo_engine_; // Normal / Reflect / Black -- see XemaEngine comment above the include

	// State
	bool connected_ = false;
	std::atomic<bool> busy_{ false };               // true while connecting (guards against overlapping connect attempts)
	std::atomic<bool> capturing_{ false };           // true while the continuous capture loop should keep running
	std::atomic<bool> capture_thread_active_{ false }; // true for the actual lifetime of captureLoopThreadFunc -- distinct from capturing_, which is just the "should keep running" request
	std::atomic<bool> applying_params_{ false };     // guards against overlapping Apply clicks
	int width_ = 0;
	int height_ = 0;
	int projector_version_ = 0; // 3010 or 4710, from DfGetProjectorVersion -- determines the real exposure range
	int pixel_type_ = 0; // 0=Mono, 1=BayerRG8 (XemaPixelType), from DfGetCameraPixelType -- determines whether DfGetBrightnessData's output is already clean grayscale or raw Bayer mosaic that needs de-mosaicing first
	int capture_engine_ = (int)XemaEngine::Normal; // UI defaults to Normal (not the SDK's raw Reflect default) -- see combo_engine_ setup in the .cpp for why

	// Raw, unprocessed copy of the last successful DfGetBrightnessData result -- kept
	// separately from the overexposure/board-marked overlay so "保存当前帧" writes exactly
	// what the SDK returned, with nothing drawn on top, for real inspection/debugging.
	cv::Mat last_raw_gray_;
	std::mutex last_raw_gray_mutex_;
};