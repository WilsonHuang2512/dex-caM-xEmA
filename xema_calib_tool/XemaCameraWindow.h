#pragma once

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QImage>
#include <QStringList>
#include <QRadioButton>
#include <QButtonGroup>
#include <QProcess>
#include <QTimer>
#include <atomic>
#include <string>
#include <vector>
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
// IMPORTANT: exposure/gain for THIS capture path (DfCaptureData + DfGetBrightnessData) are
// DfSetParamCameraExposure(float) and DfSetParamCameraGain(float) -- confirmed from the real
// vendor GUI's do_spin_camera_exposure_changed / do_doubleSpin_gain handlers, which call
// exactly these two. DfGetBrightnessData -- the actual open_cam3d.cpp implementation this
// build links -- already does pixel-type-aware Bayer->RGB->Gray conversion internally when
// pixel_type_ is BayerRG8. pixel_type_ is still tracked/logged (diagnostic info shown in the
// connect log) but doesn't drive a separate capture code path.
#include "../sdk/open_cam3d.h"
#include "../sdk/xema_enums.h"

// Capture engine locked to Black (from XEMA-master/sdk/open_cam3d.h's XemaEngine enum:
// Normal=0, Reflect=1, Black=2). DfCaptureData branches internally on this -- Normal uses
// DfGetFrame04, Reflect uses DfGetFrame06, Black uses DfGetFrame06Mono12 -- genuinely
// different reconstruction paths. Board recognition was confirmed working with the real
// vendor GUI set to Black, so this tool now always forces Black too (DfSetCaptureEngine
// called once right after connecting, no UI to change it, no persistence).

// Board spacing options -- passed straight through as calibration.exe's --board argument.
// Doesn't drive any detection here (there is none) -- purely a CLI parameter.
enum class XemaBoardSpacingMm
{
	Spacing4 = 4,
	Spacing12 = 12,
	Spacing20 = 20,
	Spacing40 = 40,
	Spacing80 = 80,
};

// Minimal XEMA camera control GUI: connect, adjust LED/gain/exposure, continuously capture
// grayscale frames, capture calibration poses, run calibration.exe, write params back to the
// camera, disconnect.
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
	void calibCaptureFinished(QString message, QImage image);
	void calibrateFinished(QString message, bool ok);
	void writeParamsFinished(QString message, bool ok);

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
	void onCaptureForCalibClicked();
	void onCalibCaptureFinished(QString message, QImage image);
	void onCalibrateClicked();
	void onCalibrateFinished(QString message, bool ok);
	void onWriteParamsClicked();
	void onWriteParamsFinished(QString message, bool ok);
	void onBrowseSavePathClicked();
	void onBusyHeartbeat();

private:
	// Status console: a real Windows console window opened alongside the GUI. Unlike the
	// in-GUI log box (short, curated status lines only), the console gets the FULL verbose
	// detail -- raw stdout from calibration.exe/open_cam3d.exe, connection trace lines, etc.
	// via logConsoleOnly(), which is thread-safe (no Qt widget touched) and callable directly
	// from background threads without waiting for a signal round-trip. log() writes to both.
	void initStatusConsole();
	static QString colorizeForConsole(const QString& msg);
	void logConsoleOnly(const QString& msg); // full verbose detail -- console only, thread-safe
	void log(const QString& msg);            // curated summary -- GUI box + console
	void* console_handle_ = nullptr;
	bool console_spinner_dirty_ = false; // true if the last console write was a \r spinner overwrite with no trailing \n -- next real log line needs a newline first

	// Busy spinner: while a long-running exe is in flight, a QTimer drives a console |/-\
	// overwrite. GUI stays a plain static status line (set once by startBusyHeartbeat) --
	// no animation in the GUI itself, only in the real console window.
	void startBusyHeartbeat(QLabel* label, const QString& prefix);
	void stopBusyHeartbeat();
	QTimer busy_heartbeat_timer_;
	int spinner_index_ = 0;
	QLabel* busy_label_ = nullptr;
	QString busy_prefix_;

	void setConnectedUiState(bool connected);
	void connectThreadFunc(QString ip);
	void applyExposureRangeForProjector(); // sets spin_exposure_'s range based on projector_version_ -- see .cpp for the real 3010/4710 bounds, confirmed from camera_capture_gui.cpp's setCameraConfigParam()
	void logCurrentParamsInto(const QString& context, QStringList& lines_out); // reads LED/exposure/gain straight from the camera and appends log lines to lines_out -- doesn't touch the GUI directly, so it's safe to call from a background thread

	// Runs continuously on a background thread while capturing_ is true: blocking-captures
	// one grayscale frame, emits captureFinished, then immediately captures the next one. No
	// fixed-interval timer -- each capture is blocking over the network already, so
	// back-to-back calls naturally throttle to the camera's real capture speed. Stops itself
	// once capturing_ goes false (checked between frames, so it exits after the in-flight
	// capture finishes, not mid-call). Sets capture_thread_active_ true on entry, false just
	// before returning -- that's how applyParamsThreadFunc knows the in-flight capture has
	// actually finished, not just that a stop was requested.
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

	// "Capture for calib" -- runs the SAME external exe the original python tool used
	// (open_cam3d.exe --get-raw-02 --ip <ip> --path <folder>), NOT the in-process
	// DfCaptureData/DfGetBrightnessData path the continuous preview uses. --get-raw-02
	// captures the full raw pattern sequence (phase00.bmp..phase36.bmp) that calibration.exe
	// actually needs -- a single brightness frame isn't enough data to calibrate from.
	// Because that exe opens its OWN connection to the camera (the camera very likely only
	// accepts one client at a time), this: (1) stops continuous capture first and waits for
	// it to actually finish, (2) re-confirms LED/exposure/gain on the camera one more time
	// (these Set calls are short-lived, connectionless-style commands, not tied to our
	// DfConnect session staying open), (3) disconnects our in-process session, (4) runs the
	// exe and waits for it to finish, (5) reconnects afterward and re-applies the Black
	// engine + current LED/gain/exposure (a fresh connect doesn't remember any of that), (6)
	// auto-resumes continuous capture if it was running before. Loads back phase36.bmp (the
	// confirmed preview frame index, from main_xema_color.py's own loadImage()) afterward
	// just to show on screen -- no detection/annotation on it anymore.
	void captureForCalibThreadFunc(QString ip, QString save_folder, int led, float gain, float exposure);

	// "Calibrate" -- runs calibration.exe against every numbered pose folder saved so far,
	// exactly matching main_xema_color.py's calibrate():
	//   calibration.exe --calibrate --use patterns-c --path <identity_folder>/
	//     --version <3010|4710> --board <spacing_mm> --calib <identity_folder>/param.txt
	// calibration.exe itself scans the bare-numbered subfolders (00, 01, ...) under
	// --path for pose data -- it does its own multi-pose discovery, we don't enumerate
	// poses ourselves. Pure file-based computation: no camera connection involved at all,
	// so unlike captureForCalibThreadFunc this does NOT touch capturing_/connected_ --
	// continuous capture (if running) is completely unaffected and keeps streaming through
	// a calibration run. --version uses the auto-detected projector_version_ (from
	// DfGetProjectorVersion at connect) rather than a manual selector, since we already know
	// it and a mismatched manual pick would silently produce a wrong calibration.
	void calibrateThreadFunc(QString identity_folder, int projector_version, int board_spacing_mm);

	// "Write params" -- runs open_cam3d.exe --set-calib-looktable --ip <ip> --path
	// <identity_folder>/param.txt, exactly matching main_xema_color.py's writeparam().
	// Requires a param.txt to already exist (i.e. Calibrate must have succeeded first).
	// Same exclusive-connection situation as captureForCalibThreadFunc -- this exe opens its
	// own connection to the camera -- so it follows the identical stop/disconnect/run/
	// reconnect/reapply/resume shape. NOTE: main_xema_color.py's writeparam() has a
	// commented-out automatic SSH reboot after writing; that's left disabled here too,
	// matching the currently-active python behavior -- add it only if explicitly asked for.
	void writeParamsThreadFunc(QString ip, QString calib_path, int led, float gain, float exposure);

	// Runs an external command to completion (blocking, with a timeout), returning its exit
	// code and capturing its combined stdout+stderr into out_output. Working directory is
	// always this tool's own exe folder -- open_cam3d.exe/calibration.exe are expected to
	// live there (with their DLLs) since there's no separate tools-folder picker anymore;
	// otherwise a process can start fine but immediately die with STATUS_DLL_NOT_FOUND
	// (0xC0000135 / exit code -1073741515), a lesson from the earlier scan tool.
	int runExeBlocking(const QString& program, const QStringList& args, QString& out_output, int timeout_ms = 60000);

	QImage grayToQImage(const cv::Mat& gray); // no more BGR overlay conversion needed -- board/overexposure marking is gone
	void loadConfig();
	void saveConfig();

	// Sanitizes edit_identity_'s text into something safe to use as a folder name (replaces
	// ':' and other separator-unsafe characters with '_') -- same convention as the original
	// scan tool's getCapturePath(), since identity strings are commonly MAC addresses.
	QString identityFolderName() const;

	// <edit_save_path_ (or exe folder if blank)>/<identityFolderName()> -- the shared root
	// both "Capture for calib" and "Calibrate"/"Write params" read from, so they always agree
	// on where poses and param.txt live without each recomputing it slightly differently.
	QString currentIdentityFolder() const;

	static const int kCaptureStopTimeoutMs = 8000; // how long applyParamsThreadFunc waits for an in-flight capture to finish before giving up and applying anyway

	// UI
	QLineEdit* edit_ip_;
	QPushButton* btn_connect_;
	QPushButton* btn_disconnect_;
	QLabel* label_firmware_;

	QLineEdit* edit_identity_;   // "标识" -- e.g. a MAC address or device label, used as the pose-folder name under edit_save_path_
	QLineEdit* edit_save_path_;  // "保存路径" -- root folder for calib poses; defaults to the exe's own folder if left blank
	QPushButton* btn_browse_save_path_;

	QSpinBox* spin_led_;
	QDoubleSpinBox* spin_gain_;
	QDoubleSpinBox* spin_exposure_;
	QPushButton* btn_apply_params_;

	QPushButton* btn_capture_; // toggles continuous capture on/off
	QPushButton* btn_capture_calib_; // stops continuous capture (if running), takes one deliberate frame, saves it as a numbered calib pose
	QPushButton* btn_calibrate_; // runs calibration.exe against all saved poses -- see calibrateThreadFunc comment
	QLabel* label_calib_status_;
	QPushButton* btn_write_params_; // writes param.txt to the camera -- see writeParamsThreadFunc comment
	QLabel* label_write_status_;
	QLabel* label_image_;
	QLabel* log_view_; // single-line status bar, like the earlier scan tool's label_status_ -- brief info only, elided if too long

	QButtonGroup* group_board_spacing_;
	QRadioButton* radio_spacing_4_;
	QRadioButton* radio_spacing_12_;
	QRadioButton* radio_spacing_20_;
	QRadioButton* radio_spacing_40_;
	QRadioButton* radio_spacing_80_;
	XemaBoardSpacingMm board_spacing_ = XemaBoardSpacingMm::Spacing80;

	// State
	bool connected_ = false;
	std::atomic<bool> busy_{ false };               // true while connecting (guards against overlapping connect attempts)
	std::atomic<bool> capturing_{ false };           // true while the continuous capture loop should keep running
	std::atomic<bool> capture_thread_active_{ false }; // true for the actual lifetime of captureLoopThreadFunc -- distinct from capturing_, which is just the "should keep running" request
	std::atomic<bool> applying_params_{ false };     // guards against overlapping Apply clicks
	std::atomic<bool> calib_capturing_{ false };     // guards against overlapping "Capture for calib" clicks
	std::atomic<bool> calibrating_{ false };         // guards against overlapping "Calibrate" clicks
	std::atomic<bool> writing_params_{ false };      // guards against overlapping "Write params" clicks
	int width_ = 0;
	int height_ = 0;
	int projector_version_ = 0; // 3010 or 4710, from DfGetProjectorVersion -- determines the real exposure range
	int pixel_type_ = 0; // 0=Mono, 1=BayerRG8 (XemaPixelType), from DfGetCameraPixelType -- diagnostic only, logged on connect
	int calib_pose_index_ = 0; // increments each successful "capture for calib" this session -- not persisted, resets on reconnect
};