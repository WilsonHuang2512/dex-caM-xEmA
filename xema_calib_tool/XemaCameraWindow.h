#pragma once

#include <windows.h>
#include <cwchar>
#include <dwmapi.h>
#pragma comment(lib, "dwmapi.lib")

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
#include <QStackedWidget>
#include <QListWidget>
#include <QComboBox>
#include <QTabWidget>
#include <QSet>
#include <QMouseEvent>
#include <atomic>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

class QVBoxLayout; // forward declaration -- only used as a pointer parameter (buildTitleBar), no need for the full header here

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

// Which calibration.exe --use variant to run. Capture (--get-raw-02) is identical either
// way -- confirmed from XEMA-master/calibration/calibration.cpp: "patterns" routes to
// calibrate_stereo() (mono board detection), "patterns-c" routes to
// calibrate_stereo_color() (color board detection). Only the calibrate step differs.
enum class XemaCalibMode
{
    Color, // --use patterns-c
    Mono,  // --use patterns
};

// Console log levels. Replaces the old approach of grepping the message text for Chinese
// substrings ("失败"/"警告"/etc) to guess a color -- the caller now states the level
// explicitly, which also drives the "[INFO]/[WARN]/[ERROR]/[EXEC]" tag written to the console.
enum class XemaLogLevel
{
	Info,    // normal status
	Success, // notable positive outcome -- tagged [INFO], colored green
	Warn,    // recoverable problem, degraded result, or a value that didn't take effect
	Error,   // operation failed
	Exec,    // subprocess invocation / raw subprocess output block markers
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

	// Rescales preview_current_image_ live when preview_window_image_label_ is resized (see
	// installEventFilter() call in the constructor), and handles mouse-wheel pose navigation --
	// so the pose photo actually fills the label when the main window is resized instead of
	// staying pinned at whatever size it was when first loaded.
	bool eventFilter(QObject* watched, QEvent* event) override;

protected:
	// Custom Win98-style caption bar drag support. The real OS title bar can't be reskinned on
	// Windows 10/11 once Qt::FramelessWindowHint is set (see buildTitleBar()), so this window
	// draws its own -- which means it's also responsible for its own drag-to-move, since the
	// native caption-drag behavior goes away along with the native frame. No manual edge-resize
	// handling -- window is a fixed frame, drag-move only via the caption bar.
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;

signals:
	// Every "Finished" signal now carries TWO strings for one event: guiMsg is short and in
	// Chinese (matches the rest of the UI), consoleMsg is the same result phrased as a clean
	// English sentence for the console. They're deliberately independent -- see log().
	void connectFinished(bool ok, QString guiMsg, QString consoleMsg, QString firmwareVersion);
	void captureFinished(bool ok, QString guiMsg, QString consoleMsg, QImage image);
	void applyParamsFinished(QString guiMsg, QString consoleMsg, bool warned);
	void disconnectFinished(QString guiMsg, QString consoleMsg, bool warned);
	void calibCaptureFinished(bool ok, QString guiMsg, QString consoleMsg, QImage image, QString poseLabel);
	void calibrateFinished(QString guiMsg, QString consoleMsg, bool ok);
	void writeParamsFinished(QString guiMsg, QString consoleMsg, bool ok);
	// "较正" (correct) -- refines an already-existing param.txt using a fresh
	// open_cam3d.exe --get-calib-param readback + calibration.exe --correct pass. See
	// correctThreadFunc for the full sequence.
	void correctFinished(QString guiMsg, QString consoleMsg, bool ok);

private slots:
	void onConnectClicked();
	void onDisconnectClicked();
	void onApplyParamsClicked();
	void onCaptureToggled();
	void onConnectFinished(bool ok, QString guiMsg, QString consoleMsg, QString firmwareVersion);
	void onCaptureFinished(bool ok, QString guiMsg, QString consoleMsg, QImage image);
	void onApplyParamsFinished(QString guiMsg, QString consoleMsg, bool warned);
	void onDisconnectFinished(QString guiMsg, QString consoleMsg, bool warned);
	void onBoardSpacingChanged();
	void onCalibModeChanged();
	void onCaptureForCalibClicked();
	void onCalibCaptureFinished(bool ok, QString guiMsg, QString consoleMsg, QImage image, QString poseLabel);
	void onCalibrateClicked();
	void onCalibrateFinished(QString guiMsg, QString consoleMsg, bool ok);
	void onWriteParamsClicked();
	void onWriteParamsFinished(QString guiMsg, QString consoleMsg, bool ok);
	void onCorrectClicked();
	void onCorrectFinished(QString guiMsg, QString consoleMsg, bool ok);
	void onBrowseSavePathClicked();
	void onBusyHeartbeat();
	void onBrowsePosesClicked(); // toggles preview_stack_ between live view and pose browse view (was "open a window" -- now a same-window toggle)
	void onPreviewPoseListRowChanged(int row);
	void onPreviewPrevClicked();
	void onPreviewNextClicked();
	void onPreviewRefreshClicked();

private:
	// Status console: a real Windows console window opened alongside the GUI. The two outputs
	// now deliberately carry DIFFERENT text, not a copy of the same string:
	//   - GUI (log_view_, via log()): one short, curated Chinese status line per user action --
	//     "already accurate, but brief" per its single-line elided design.
	//   - Console (via logConsoleOnly()): a full English, leveled ([INFO]/[WARN]/[ERROR]/[EXEC])
	//     trace, including live step-by-step progress from background threads (disconnect-for-
	//     exe, exe invocation + raw stdout, reconnect, etc) that would flood the GUI line if
	//     shown there. logConsoleOnly() is thread-safe (WriteFile to a console handle touches no
	//     Qt widget) and callable directly from background threads without a signal round-trip.
	// log() is only ever called from the GUI thread (mirrors the final result of a user action
	// into both places); logConsoleOnly() is used for everything in between.
	void initStatusConsole();

	// Builds the hand-drawn Win98 caption bar (navy gradient, white bold title, Marlett-glyph
	// min/max/close buttons) and adds it as main_layout's first item, flush to the window edge.
	// Called once from the constructor, before the rest of the UI is built.
	void buildTitleBar(QVBoxLayout* main_layout);
	QWidget* title_bar_ = nullptr;
	QLabel* title_bar_label_ = nullptr;
	QPushButton* btn_min_ = nullptr;
	QPushButton* btn_max_ = nullptr;
	QPushButton* btn_close_ = nullptr;
	bool dragging_ = false;
	QPoint drag_offset_; // mouse pos relative to the window's own top-left, captured once at drag-press time

	static QString levelTag(XemaLogLevel level);   // "[INFO]" / "[WARN]" / "[ERROR]" / "[EXEC]"
	static QString colorizeForConsole(XemaLogLevel level, const QString& tagged_msg, bool highlight);
	// highlight=true renders the line bold, for lines worth the reader's eye even in a wall of
	// exe output (board-recognition summary, reprojection error) -- see logCalibExeOutput().
	void logConsoleOnly(XemaLogLevel level, const QString& console_text, bool highlight = false);
	void log(XemaLogLevel level, const QString& gui_text, const QString& console_text); // GUI box + console
	void* console_handle_ = nullptr;
	bool console_spinner_dirty_ = false; // true if the last console write was a \r spinner overwrite with no trailing \n -- next real log line needs a newline first

	// Prints calibration.exe's raw stdout in FULL (nothing hidden -- scroll up in the console
	// for line-by-line detail), then a bolded summary block underneath pulling out just the
	// lines that matter (board image count, reprojection error, pass/fail; repeated "found"
	// lines collapsed into one count) so the takeaway is visible without scrolling. `tag`
	// labels the bracketed section headers ("[calib]" by default, "[correct]" when called from
	// correctThreadFunc) -- both --calibrate and --correct share the same stdout format.
	void logCalibExeOutput(const QString& raw_output, const QString& tag = "calib");

	// calibration.exe's stdout never names WHICH pose failed board detection ("found" is
	// printed with no pose number attached) -- but it writes "<N>_board.bmp" for every pose it
	// processed and "<N>_draw.bmp" (an annotated visualization) ONLY for poses where the board
	// was actually found. Diffing those two sets on disk recovers exactly which poses need to
	// be recaptured. Confirmed by sb from a live folder listing (2026-08-24) -- board.bmp always
	// present, draw.bmp only on success. Logs a highlighted [WARN] listing the missing pose
	// numbers (zero-padded to match the capture folder naming, e.g. "07"), or a highlighted
	// [INFO] confirming all poses were detected if none are missing. Called as part of the
	// same summary block logCalibExeOutput prints, so it's the last thing before the result.
	// `tag` matches whatever logCalibExeOutput was called with for the same run.
	void logMissingBoardPoses(const QString& identity_folder, const QString& tag = "calib");

	// Pose browsing lives in the SAME area as the live capture preview now (not a separate
	// window) -- preview_stack_ is a QStackedWidget with two pages: page 0 is label_image_
	// (today's live feed, untouched), page 1 is the pose-browse page (list + big image) built
	// once here at construction time. btn_browse_poses_ ("浏览已拍照片") just flips
	// preview_stack_'s current page; nothing is created/destroyed on toggle. Deliberately does
	// NOT auto-switch to the browse page after a fresh capture (that would yank the live feed
	// away mid-session, e.g. while rapid-capturing several poses back to back) -- capturing
	// just quietly keeps the pose list's data current in the background (see
	// refreshPreviewPoseListIfVisible()) so switching to it later shows the latest.
	//
	// Two DIFFERENT data sources, deliberately not the same file:
	//   - List color (grey/black): calibration.exe's own diagnostic output -- "<N>_draw.bmp"
	//     (root of identity_folder, no zero-padding) existing means the board WAS detected for
	//     that pose in the most recent calibration run. No draw.bmp = grey, whether that's
	//     because the board wasn't found or calibration.exe simply hasn't been run yet. This
	//     only updates when a calibration run actually finishes (see onCalibrateFinished) --
	//     capturing a pose does NOT touch its color, only its checkmark (below).
	//   - Displayed image: phase36.bmp (identity_folder/<pose>/phase36.bmp, INSIDE the pose's
	//     own zero-padded subfolder) -- the confirmed preview frame index from
	//     main_xema_color.py's own loadImage() (the original Python tool). Available the moment
	//     a pose is captured, well before any calibration has run, so you can always see what
	//     was actually photographed regardless of whether it's been verified yet.
	//   - Checkmark ("✓" appended to the label): tracks recaptured_poses_ -- a pose you just
	//     (re)captured since the last calibration run, so its grey/black color is stale and
	//     hasn't been re-verified yet. Cleared entirely the next time Calibrate finishes, at
	//     which point the color reflects the fresh result instead.
	// Navigate by clicking a list entry or scrolling the mouse wheel over the image (see
	// eventFilter()) -- no Prev/Next buttons, no position readout, just the list + the image.
	void refreshPreviewPoseList(); // rescans preview_identity_folder_ for numbered pose subfolders, tries to keep the current selection
	void refreshPreviewPoseListIfVisible(); // same, but only if preview_stack_ is currently showing the browse page -- called after each pose capture and after each calibration run so it doesn't do pointless work while the live page is showing
	void loadPreviewPoseAt(int row); // loads phase36.bmp for preview_pose_labels_[row] into the big image and updates list selection
	QStackedWidget* preview_stack_ = nullptr;
	QListWidget* preview_pose_list_ = nullptr;
	QLabel* preview_window_image_label_ = nullptr; // name kept from the old popup-window design -- it's page 1 of preview_stack_ now, not a separate window
	QPushButton* preview_refresh_btn_ = nullptr;
	QPushButton* btn_browse_poses_ = nullptr; // toggles preview_stack_ between live view and pose browse view
	QString preview_identity_folder_; // identity folder the list was last scanned from
	QStringList preview_pose_labels_; // sorted pose folder names currently listed, e.g. "00","01",...
	int preview_current_row_ = -1; // index into preview_pose_labels_ currently displayed, -1 if none
	QImage preview_current_image_; // kept so eventFilter() can rescale it live when the label is resized
	QSet<QString> recaptured_poses_; // pose labels (re)captured since the last calibration run -- shown with a "✓" in the list, cleared on the next calibrate

	// Busy indicator: matches ScanToolWindow's updateBusyIndicator() format exactly --
	//   - Console: a classic \r-overwritten |/-\ spinner, tagged with the same bracketed
	//     prefix as the messages around it (e.g. "[calib] working"), colored bright cyan.
	//   - GUI (log_view_, the terminal-styled status box -- our equivalent of ScanToolWindow's
	//     label_status_): a single block character (U+2588) bouncing back and forth across a
	//     12-char '.'-filled track, e.g. "[calib] working  [..#.......]", overwriting log_view_
	//     live.
	// prefix should be a bracketed tag matching the operation, e.g. "[calib] working" /
	// "[write] working", same convention ScanToolWindow uses ("[calib] ...", "[connect] ...").
	void startBusyHeartbeat(const QString& prefix);
	void stopBusyHeartbeat();
	QTimer busy_heartbeat_timer_;
	int busy_spinner_index_ = 0;
	QString busy_prefix_;

	void setConnectedUiState(bool connected);
	void connectThreadFunc(QString ip);
	void applyExposureRangeForProjector(); // sets spin_exposure_'s range based on projector_version_ -- see .cpp for the real 3010/4710 bounds, confirmed from camera_capture_gui.cpp's setCameraConfigParam()

	// Reads LED/exposure/gain straight from the camera and writes an English readback line
	// (plus a [WARN] line per failed read) straight to the console via logConsoleOnly() --
	// doesn't touch the GUI, so it's safe to call from a background thread. `context` is a
	// short English tag identifying when this readback happened, e.g. "post-connect".
	void logCurrentParamsInto(const QString& context);

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
	void captureForCalibThreadFunc(QString ip, QString save_folder, QString pose_label, int led, float gain, float exposure);

	// "Calibrate" -- runs calibration.exe against every numbered pose folder saved so far,
	// matching main_xema_color.py's calibrate(), except --use is now selectable instead of
	// hardcoded:
	//   calibration.exe --calibrate --use <patterns|patterns-c> --path <identity_folder>/
	//     --version <3010|4710> --board <spacing_mm> --calib <identity_folder>/param.txt
	// calibration.exe itself scans the bare-numbered subfolders (00, 01, ...) under
	// --path for pose data -- it does its own multi-pose discovery, we don't enumerate
	// poses ourselves. Pure file-based computation: no camera connection involved at all,
	// so unlike captureForCalibThreadFunc this does NOT touch capturing_/connected_ --
	// continuous capture (if running) is completely unaffected and keeps streaming through
	// a calibration run. --version uses the auto-detected projector_version_ (from
	// DfGetProjectorVersion at connect) rather than a manual selector, since we already know
	// it and a mismatched manual pick would silently produce a wrong calibration.
	void calibrateThreadFunc(QString identity_folder, int projector_version, int board_spacing_mm, XemaCalibMode calib_mode);

	// "Write params" -- runs open_cam3d.exe --set-calib-looktable --ip <ip> --path
	// <identity_folder>/param.txt, exactly matching main_xema_color.py's writeparam().
	// Requires a param.txt to already exist (i.e. Calibrate must have succeeded first).
	// Same exclusive-connection situation as captureForCalibThreadFunc -- this exe opens its
	// own connection to the camera -- so it follows the identical stop/disconnect/run/
	// reconnect/reapply/resume shape. NOTE: main_xema_color.py's writeparam() has a
	// commented-out automatic SSH reboot after writing; that's left disabled here too,
	// matching the currently-active python behavior -- add it only if explicitly asked for.
	void writeParamsThreadFunc(QString ip, QString calib_path, int led, float gain, float exposure);

	// "较正" (Correct) -- refines an EXISTING param.txt using new capture patterns, instead of
	// calibrating one from scratch. Matches main_xema_correct_color.py's calibrate() (its
	// button is labeled "标定" in that tool but it's really a correction pass, not a fresh
	// calibrate -- confirmed from its command: calibration.exe --correct, not --calibrate):
	//   1. open_cam3d.exe --get-calib-param --ip <ip> --path <param_in_path> -- fetches the
	//      parameters CURRENTLY on the camera into a staging file (same one-client-at-a-time
	//      situation as --get-raw-02/--set-calib-looktable, so this also needs the
	//      disconnect/run/reconnect dance).
	//   2. calibration.exe --correct --use <patterns|patterns-c> --path <identity_folder>/
	//      --version <3010|4710> --board <spacing_mm> --param-in <param_in_path>
	//      --param-out <identity_folder>/param.txt -- reads the same captured pose patterns
	//      Calibrate would use, but refines the fetched params instead of computing fresh
	//      ones. This half is pure file-based computation (no camera needed), so it runs
	//      AFTER reconnecting/resuming, unlike step 1.
	void correctThreadFunc(QString ip, QString identity_folder, int projector_version, int board_spacing_mm, XemaCalibMode calib_mode, int led, float gain, float exposure);

	// Runs an external command to completion (blocking, with a timeout), returning its exit
	// code and capturing its combined stdout+stderr into out_output. Working directory is
	// always this tool's own exe folder -- open_cam3d.exe/calibration.exe are expected to
	// live there (with their DLLs) since there's no separate tools-folder picker anymore;
	// otherwise a process can start fine but immediately die with STATUS_DLL_NOT_FOUND
	// (0xC0000135 / exit code -1073741515), a lesson from the earlier scan tool.
	int runExeBlocking(const QString& program, const QStringList& args, QString& out_output, int timeout_ms = 60000);

	QImage grayToQImage(const cv::Mat& gray);
	QImage colorToQImage(const cv::Mat& bgr); // for this tool's own live-preview overlay (annotateLiveFrame) -- color-annotated (BGR) images, unlike the plain grayscale phase36.bmp/board captures

	// Always-on live overlay, applied to every frame in captureLoopThreadFunc -- not a toggle,
	// not on-demand:
	//   - Overexposure: marks any pixel >= 254 red. 254 matches the firmware's own threshold
	//     (configure_auto_exposure.cpp's evaluateBrightnessParam: cv::threshold(img,
	//     over_exposure_mask, 254, 255, ...)), not the same as fully-saturated 255.
	//   - Board detection: draws the calibration board's detected points in green if found,
	//     using the SAME algorithm calibration.exe's own Calibrate_Function::
	//     findCircleBoardFeature() uses (confirmed independently in calibrate_function.cpp,
	//     precision_test.cpp, and configure_standard_plane.cpp: invert the image,
	//     cv::findCirclesGrid with CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING, board size
	//     7x11 -- constant across all spacings per precision_test.cpp's getBoard()).
	// Runs on every live frame, so it's real-time feedback while adjusting exposure/gain and
	// positioning the board -- not a per-frame log line (that would flood the console).
	QImage annotateLiveFrame(const cv::Mat& gray);

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
	// "起始姿态编号" -- which pose slot "拍照（用于标定）" saves into next, same mechanism as
	// ScanToolWindow's combo_group_: onCaptureForCalibClicked reads currentIndex() directly as
	// the pose number (item text IS the zero-padded index, so index and folder name always
	// agree), and a successful capture advances it by one afterward (capped at the last item),
	// exactly like ScanToolWindow::onShotClicked. Picking an earlier number here and capturing
	// is how you deliberately overwrite/recapture a specific pose (e.g. one that came back grey
	// after a calibration run) instead of only ever appending new ones.
	QComboBox* combo_start_pose_;
	QPushButton* btn_capture_calib_; // stops continuous capture (if running), takes one deliberate frame, saves it as a numbered calib pose
	QPushButton* btn_calibrate_; // runs calibration.exe against all saved poses -- see calibrateThreadFunc comment
	QPushButton* btn_write_params_; // writes param.txt to the camera -- see writeParamsThreadFunc comment
	QPushButton* btn_correct_ = nullptr; // refines an existing param.txt using new patterns -- see correctThreadFunc comment
	QLabel* label_image_;
	QLabel* log_view_; // single-line status bar, like the earlier scan tool's label_status_ -- brief info only, elided if too long

	QButtonGroup* group_board_spacing_;
	QRadioButton* radio_spacing_4_;
	QRadioButton* radio_spacing_12_;
	QRadioButton* radio_spacing_20_;
	QRadioButton* radio_spacing_40_;
	QRadioButton* radio_spacing_80_;
	XemaBoardSpacingMm board_spacing_ = XemaBoardSpacingMm::Spacing80;

	QButtonGroup* group_calib_mode_ = nullptr;
	QRadioButton* radio_calib_color_ = nullptr;
	QRadioButton* radio_calib_mono_ = nullptr;
	XemaCalibMode calib_mode_ = XemaCalibMode::Color; // matches current hardcoded behavior as default

	// State
	bool connected_ = false;
	std::atomic<bool> busy_{ false };               // true while connecting (guards against overlapping connect attempts)
	std::atomic<bool> capturing_{ false };           // true while the continuous capture loop should keep running
	std::atomic<bool> capture_thread_active_{ false }; // true for the actual lifetime of captureLoopThreadFunc -- distinct from capturing_, which is just the "should keep running" request
	std::atomic<bool> applying_params_{ false };     // guards against overlapping Apply clicks
	std::atomic<bool> calib_capturing_{ false };     // guards against overlapping "Capture for calib" clicks
	std::atomic<bool> calibrating_{ false };         // guards against overlapping "Calibrate" clicks
	std::atomic<bool> writing_params_{ false };      // guards against overlapping "Write params" clicks
	std::atomic<bool> correcting_{ false };          // guards against overlapping "Correct" clicks
	int width_ = 0;
	int height_ = 0;
	int projector_version_ = 0;
	QString firmware_version_ = u8"-"; // e.g. "v1.5.5", from DfGetFirmwareVersion at connect time -- reused verbatim on reconnects (calib capture / write params) so the badge never regresses to a placeholder
	int pixel_type_ = 0; // 0=Mono, 1=BayerRG8 (XemaPixelType), from DfGetCameraPixelType -- diagnostic only, logged on connect
};