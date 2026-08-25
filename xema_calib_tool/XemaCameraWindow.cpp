#include "XemaCameraWindow.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QApplication>
#include <QCoreApplication>
#include <QFile>
#include <QDir>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <QFontMetrics>
#include <QEvent>
#include <QColor>
#include <QWheelEvent>
#include <thread>
#include <cmath>
#include <algorithm>
#include <QElapsedTimer>
#include <QAbstractButton>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <cwchar>
#endif

// Same Win98-style theme as the earlier scan tool, so the two tools look/feel consistent.
static const char* kWin98Style =
	"QWidget { background-color: #c0c0c0; color: #000000; font-family: 'Tahoma','MS Sans Serif','Segoe UI'; font-size: 9pt; }"
	"QPushButton { background-color: #c0c0c0; border-style: outset; border-width: 2px;"
	"  border-color: #ffffff #808080 #808080 #ffffff; padding: 4px 10px; }"
	"QPushButton:pressed { border-style: inset; border-color: #808080 #ffffff #ffffff #808080; }"
	"QPushButton:disabled { color: #808080; }"
	"QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {"
	"  background-color: #ffffff; border-style: inset; border-width: 2px;"
	"  border-color: #808080 #ffffff #ffffff #808080; padding: 2px; }"
	"QGroupBox { border: 2px groove #808080; margin-top: 10px; padding-top: 6px; font-weight: bold; }"
	"QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }";

static const char* kTerminalLogStyle =
	"background-color: #000000; color: #33ff66; border-style: inset; border-width: 2px;"
	"border-color: #808080 #ffffff #ffffff #808080; padding: 3px 6px;"
	"font-family: 'Consolas','Lucida Console',monospace; font-size: 9pt;";

XemaCameraWindow::XemaCameraWindow(QWidget* parent)
	: QWidget(parent)
{
	initStatusConsole();

	setWindowTitle(u8"XEMA 相机控制");
	setStyleSheet(kWin98Style);
	resize(700, 700);

	// ---- connection row ----
	edit_ip_ = new QLineEdit(this);
	edit_ip_->setPlaceholderText(u8"相机IP，如 192.168.15.139");
	btn_connect_ = new QPushButton(u8"连接", this);
	btn_disconnect_ = new QPushButton(u8"断开", this);
	btn_disconnect_->setEnabled(false);
	label_firmware_ = new QLabel(u8"固件版本: -", this);

	QHBoxLayout* connect_row = new QHBoxLayout();
	connect_row->addWidget(new QLabel(u8"IP:", this));
	connect_row->addWidget(edit_ip_);
	connect_row->addWidget(btn_connect_);
	connect_row->addWidget(btn_disconnect_);
	connect_row->addWidget(label_firmware_);

	edit_identity_ = new QLineEdit(this);
	edit_identity_->setPlaceholderText(u8"标识，如 MAC 地址或设备名称");
	edit_save_path_ = new QLineEdit(this);
	edit_save_path_->setPlaceholderText(u8"图像保存根目录（留空则使用程序所在目录）");
	btn_browse_save_path_ = new QPushButton(u8"浏览...", this);

	QHBoxLayout* identity_row = new QHBoxLayout();
	identity_row->addWidget(new QLabel(u8"标识:", this));
	identity_row->addWidget(edit_identity_);
	identity_row->addWidget(new QLabel(u8"保存路径:", this));
	identity_row->addWidget(edit_save_path_);
	identity_row->addWidget(btn_browse_save_path_);

	// ---- params ----
	spin_led_ = new QSpinBox(this);
	spin_led_->setRange(0, 1023);
	spin_led_->setSingleStep(100); // up/down arrows step by 100
	spin_led_->setValue(1023);

	spin_gain_ = new QDoubleSpinBox(this);
	spin_gain_->setRange(0.0, 24.0); // matches the real GUI's doubleSpinBox_gain range
	spin_gain_->setSingleStep(1.0); // up/down arrows step by 1
	spin_gain_->setValue(0.0);

	spin_exposure_ = new QDoubleSpinBox(this);
	// Real range depends on which projector is attached (3010: 1700-100000, 4710:
	// 1700-28000, confirmed from camera_capture_gui.cpp's setCameraConfigParam()) -- this is
	// just a placeholder until DfGetProjectorVersion comes back after connecting;
	// applyExposureRangeForProjector() narrows it to the real bounds then.
	spin_exposure_->setRange(1700.0, 100000.0);
	spin_exposure_->setDecimals(0);
	spin_exposure_->setSingleStep(1000.0); // up/down arrows step by 1000
	spin_exposure_->setValue(30000.0);

	btn_apply_params_ = new QPushButton(u8"应用参数", this);
	btn_apply_params_->setEnabled(false);

	QGroupBox* param_box = new QGroupBox(u8"参数", this);
	QHBoxLayout* param_row = new QHBoxLayout(param_box);
	param_row->addWidget(new QLabel(u8"LED 亮度:", this));
	param_row->addWidget(spin_led_);
	param_row->addWidget(new QLabel(u8"增益:", this));
	param_row->addWidget(spin_gain_);
	param_row->addWidget(new QLabel(u8"曝光时间:", this));
	param_row->addWidget(spin_exposure_);
	param_row->addWidget(btn_apply_params_);

	// ---- board spacing (still needed as calibration.exe's --board argument) ----
	group_board_spacing_ = new QButtonGroup(this);
	radio_spacing_4_ = new QRadioButton("4mm", this);
	radio_spacing_12_ = new QRadioButton("12mm", this);
	radio_spacing_20_ = new QRadioButton("20mm", this);
	radio_spacing_40_ = new QRadioButton("40mm", this);
	radio_spacing_80_ = new QRadioButton("80mm", this);
	group_board_spacing_->addButton(radio_spacing_4_);
	group_board_spacing_->addButton(radio_spacing_12_);
	group_board_spacing_->addButton(radio_spacing_20_);
	group_board_spacing_->addButton(radio_spacing_40_);
	group_board_spacing_->addButton(radio_spacing_80_);
	radio_spacing_80_->setChecked(true);

	QGroupBox* board_box = new QGroupBox(u8"标定板间距", this);
	QHBoxLayout* board_row = new QHBoxLayout(board_box);
	board_row->addWidget(radio_spacing_4_);
	board_row->addWidget(radio_spacing_12_);
	board_row->addWidget(radio_spacing_20_);
	board_row->addWidget(radio_spacing_40_);
	board_row->addWidget(radio_spacing_80_);

	// ---- capture ----
	btn_capture_ = new QPushButton(u8"开始连续采集", this);
	btn_capture_->setEnabled(false);

	// "起始姿态编号" -- same mechanism as ScanToolWindow's combo_group_: item text IS the
	// zero-padded pose number, currentIndex() doubles as the pose number directly. 50 slots
	// (00-49) is generous headroom for a normal calibration session.
	combo_start_pose_ = new QComboBox(this);
	for (int i = 0; i < 50; i++)
	{
		combo_start_pose_->addItem(QString("%1").arg(i, 2, 10, QChar('0')));
	}
	combo_start_pose_->setToolTip(u8"下一次拍照保存到的姿态编号 -- 选择已有编号即可覆盖重拍那一组");

	btn_capture_calib_ = new QPushButton(u8"拍照（用于标定）", this);
	btn_capture_calib_->setEnabled(false);
	btn_capture_calib_->setToolTip(u8"停止连续采集（如果正在运行），拍摄一张单独的标定用图像并保存编号");

	btn_browse_poses_ = new QPushButton(u8"浏览已拍照片", this);
	btn_browse_poses_->setEnabled(true); // works purely off disk -- no camera connection needed
	btn_browse_poses_->setToolTip(u8"查看「标识/保存路径」下已保存的标定姿态照片，即使本次未拍照也可以浏览之前保存的");

	QHBoxLayout* capture_row = new QHBoxLayout();
	capture_row->addWidget(btn_capture_);
	capture_row->addWidget(new QLabel(u8"起始:", this));
	capture_row->addWidget(combo_start_pose_);
	capture_row->addWidget(btn_capture_calib_);
	capture_row->addWidget(btn_browse_poses_);

	// ---- calibrate ----
	btn_calibrate_ = new QPushButton(u8"标定", this);
	btn_calibrate_->setEnabled(true); // no camera connection needed
	btn_calibrate_->setToolTip(u8"对已拍摄的标定姿态运行 calibration.exe，不需要相机连接");

	btn_write_params_ = new QPushButton(u8"写参数", this);
	btn_write_params_->setEnabled(false); // needs a live connection, like capture-for-calib
	btn_write_params_->setToolTip(u8"将标定结果 (param.txt) 写入相机 -- 需要先成功运行标定");

	QHBoxLayout* calibrate_row = new QHBoxLayout();
	calibrate_row->addWidget(btn_calibrate_);
	calibrate_row->addWidget(btn_write_params_);
	calibrate_row->addStretch();

	label_image_ = new QLabel(this);
	label_image_->setMinimumHeight(350);
	label_image_->setAlignment(Qt::AlignCenter);
	label_image_->setText(u8"（未连接）");
	label_image_->setStyleSheet("background-color:#222; color:#aaa; border: 1px solid #888;");

	// ---- pose browse page (toggled in via btn_browse_poses_ -- see header comment) ----
	preview_pose_list_ = new QListWidget(this);
	preview_pose_list_->setFixedWidth(90);

	preview_refresh_btn_ = new QPushButton(u8"刷新", this);
	preview_refresh_btn_->setToolTip(u8"重新扫描文件夹（例如切换了标识/保存路径，或用其他方式添加了姿态）");

	QVBoxLayout* pose_list_col = new QVBoxLayout();
	pose_list_col->addWidget(new QLabel(u8"姿态", this));
	pose_list_col->addWidget(preview_pose_list_, 1);
	pose_list_col->addWidget(preview_refresh_btn_);

	preview_window_image_label_ = new QLabel(this);
	preview_window_image_label_->setAlignment(Qt::AlignCenter);
	preview_window_image_label_->setMinimumHeight(350); // matches label_image_'s min height -- switching pages shouldn't resize the window
	preview_window_image_label_->setText(u8"（未找到预览图）");
	preview_window_image_label_->setStyleSheet("background-color:#222; color:#aaa; border: 1px solid #888;");
	// Rescales the currently-shown photo live when this label is resized, and lets the mouse
	// wheel step between poses while hovering it -- see eventFilter().
	preview_window_image_label_->installEventFilter(this);

	QWidget* pose_browse_page = new QWidget(this);
	QHBoxLayout* pose_browse_row = new QHBoxLayout(pose_browse_page);
	pose_browse_row->setContentsMargins(0, 0, 0, 0);
	pose_browse_row->addLayout(pose_list_col);
	pose_browse_row->addWidget(preview_window_image_label_, 1);

	preview_stack_ = new QStackedWidget(this);
	preview_stack_->addWidget(label_image_);    // page 0 -- live capture feed (default)
	preview_stack_->addWidget(pose_browse_page); // page 1 -- pose browser

	// ---- log ----
	log_view_ = new QLabel(this);
	log_view_->setFixedHeight(26);
	log_view_->setStyleSheet(kTerminalLogStyle);
	log_view_->setTextInteractionFlags(Qt::TextSelectableByMouse);

	QVBoxLayout* main_layout = new QVBoxLayout(this);
	main_layout->addLayout(connect_row);
	main_layout->addLayout(identity_row);
	main_layout->addWidget(param_box);
	main_layout->addWidget(board_box);
	main_layout->addLayout(capture_row);
	main_layout->addLayout(calibrate_row);
	main_layout->addWidget(preview_stack_, 1);
	main_layout->addWidget(log_view_);

	connect(btn_connect_, &QPushButton::clicked, this, &XemaCameraWindow::onConnectClicked);
	connect(btn_disconnect_, &QPushButton::clicked, this, &XemaCameraWindow::onDisconnectClicked);
	connect(btn_apply_params_, &QPushButton::clicked, this, &XemaCameraWindow::onApplyParamsClicked);
	connect(btn_capture_, &QPushButton::clicked, this, &XemaCameraWindow::onCaptureToggled);
	connect(btn_capture_calib_, &QPushButton::clicked, this, &XemaCameraWindow::onCaptureForCalibClicked);
	connect(btn_browse_poses_, &QPushButton::clicked, this, &XemaCameraWindow::onBrowsePosesClicked);
	connect(btn_calibrate_, &QPushButton::clicked, this, &XemaCameraWindow::onCalibrateClicked);
	connect(btn_write_params_, &QPushButton::clicked, this, &XemaCameraWindow::onWriteParamsClicked);
	connect(btn_browse_save_path_, &QPushButton::clicked, this, &XemaCameraWindow::onBrowseSavePathClicked);
	connect(group_board_spacing_, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked), this, &XemaCameraWindow::onBoardSpacingChanged);
	connect(preview_pose_list_, &QListWidget::currentRowChanged, this, &XemaCameraWindow::onPreviewPoseListRowChanged);
	connect(preview_refresh_btn_, &QPushButton::clicked, this, &XemaCameraWindow::onPreviewRefreshClicked);
	connect(this, &XemaCameraWindow::connectFinished, this, &XemaCameraWindow::onConnectFinished);
	connect(this, &XemaCameraWindow::captureFinished, this, &XemaCameraWindow::onCaptureFinished);
	connect(this, &XemaCameraWindow::applyParamsFinished, this, &XemaCameraWindow::onApplyParamsFinished);
	connect(this, &XemaCameraWindow::disconnectFinished, this, &XemaCameraWindow::onDisconnectFinished);
	connect(this, &XemaCameraWindow::calibCaptureFinished, this, &XemaCameraWindow::onCalibCaptureFinished);
	connect(this, &XemaCameraWindow::calibrateFinished, this, &XemaCameraWindow::onCalibrateFinished);
	connect(this, &XemaCameraWindow::writeParamsFinished, this, &XemaCameraWindow::onWriteParamsFinished);
	connect(&busy_heartbeat_timer_, &QTimer::timeout, this, &XemaCameraWindow::onBusyHeartbeat);

	loadConfig();
	log(XemaLogLevel::Info, u8"就绪", "Ready");
}

XemaCameraWindow::~XemaCameraWindow()
{
	capturing_ = false; // request the loop thread stop

	QElapsedTimer wait_timer;
	wait_timer.start();
	while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	if (connected_)
	{
		DfDisconnect(edit_ip_->text().trimmed().toStdString().c_str());
	}

#ifdef _WIN32
	if (console_handle_)
	{
		CloseHandle((HANDLE)console_handle_);
		console_handle_ = nullptr;
	}
	FreeConsole();
#endif
}

// ==================== config ====================

void XemaCameraWindow::loadConfig()
{
	QString path = QCoreApplication::applicationDirPath() + "/xema_camera_gui_config.json";
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
	{
		return; // no config yet -- keep the UI's built-in defaults
	}

	QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
	f.close();
	QJsonObject obj = doc.object();

	if (obj.contains("ip")) edit_ip_->setText(obj.value("ip").toString());
	if (obj.contains("led")) spin_led_->setValue(obj.value("led").toInt(spin_led_->value()));
	if (obj.contains("gain")) spin_gain_->setValue(obj.value("gain").toDouble(spin_gain_->value()));
	if (obj.contains("exposure")) spin_exposure_->setValue(obj.value("exposure").toDouble(spin_exposure_->value()));
	if (obj.contains("identity")) edit_identity_->setText(obj.value("identity").toString());
	if (obj.contains("save_path")) edit_save_path_->setText(obj.value("save_path").toString());
	// Last known-good projector model from a previous session's successful connect -- lets
	// Calibrate (which is designed to run fully offline, no camera needed) still pass the
	// right --version to calibration.exe even if the camera hasn't been connected THIS
	// session. A fresh connect below always overwrites this with the real detected value.
	if (obj.contains("projector_version")) projector_version_ = obj.value("projector_version").toInt(0);

	int spacing = obj.value("board_spacing_mm").toInt(80);
	switch (spacing)
	{
	case 4: radio_spacing_4_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing4; break;
	case 12: radio_spacing_12_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing12; break;
	case 20: radio_spacing_20_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing20; break;
	case 40: radio_spacing_40_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing40; break;
	default: radio_spacing_80_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing80; break;
	}
}

void XemaCameraWindow::saveConfig()
{
	QString path = QCoreApplication::applicationDirPath() + "/xema_camera_gui_config.json";
	QJsonObject obj;
	obj["ip"] = edit_ip_->text();
	obj["led"] = spin_led_->value();
	obj["gain"] = spin_gain_->value();
	obj["exposure"] = spin_exposure_->value();
	obj["board_spacing_mm"] = (int)board_spacing_;
	obj["identity"] = edit_identity_->text();
	obj["save_path"] = edit_save_path_->text();
	obj["projector_version"] = projector_version_;

	QFile f(path);
	if (f.open(QIODevice::WriteOnly))
	{
		f.write(QJsonDocument(obj).toJson());
		f.close();
	}
}

void XemaCameraWindow::onBoardSpacingChanged()
{
	if (radio_spacing_4_->isChecked()) board_spacing_ = XemaBoardSpacingMm::Spacing4;
	else if (radio_spacing_12_->isChecked()) board_spacing_ = XemaBoardSpacingMm::Spacing12;
	else if (radio_spacing_20_->isChecked()) board_spacing_ = XemaBoardSpacingMm::Spacing20;
	else if (radio_spacing_40_->isChecked()) board_spacing_ = XemaBoardSpacingMm::Spacing40;
	else if (radio_spacing_80_->isChecked()) board_spacing_ = XemaBoardSpacingMm::Spacing80;

	log(XemaLogLevel::Info,
		QString(u8"标定板间距: %1mm").arg((int)board_spacing_),
		QString("Board spacing set to %1 mm").arg((int)board_spacing_));
	saveConfig();
}

void XemaCameraWindow::onBrowseSavePathClicked()
{
	QString dir = QFileDialog::getExistingDirectory(this, u8"选择图像保存根目录", edit_save_path_->text());
	if (!dir.isEmpty())
	{
		edit_save_path_->setText(QDir::toNativeSeparators(dir));
		saveConfig();
	}
}

QString XemaCameraWindow::identityFolderName() const
{
	QString id = edit_identity_->text().trimmed();
	id.replace(":", "_");
	id.replace("-", "_");
	id.replace(" ", "_");
	if (id.isEmpty())
	{
		id = "default"; // still need *some* folder name if the identity field is left blank
	}
	return id;
}

QString XemaCameraWindow::currentIdentityFolder() const
{
	QString base_path = edit_save_path_->text().trimmed();
	if (base_path.isEmpty())
	{
		base_path = QCoreApplication::applicationDirPath();
	}
	return base_path + "/" + identityFolderName();
}

// ==================== status console ====================

void XemaCameraWindow::initStatusConsole()
{
#ifdef _WIN32
	if (AllocConsole())
	{
		SetConsoleTitleA("XEMA Camera GUI - Log Console");

		HANDLE h = CreateFileA("CONOUT$", GENERIC_READ | GENERIC_WRITE,
			FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
		if (h != INVALID_HANDLE_VALUE)
		{
			console_handle_ = h;

			CONSOLE_FONT_INFOEX cfi;
			ZeroMemory(&cfi, sizeof(cfi));
			cfi.cbSize = sizeof(cfi);
			cfi.dwFontSize.Y = 18;
			cfi.FontFamily = 0x04; // TMPF_TRUETYPE
			cfi.FontWeight = FW_NORMAL;
			wcscpy_s(cfi.FaceName, L"Consolas");
			SetCurrentConsoleFontEx((HANDLE)h, FALSE, &cfi);

			DWORD mode = 0;
			if (GetConsoleMode((HANDLE)h, &mode))
			{
				SetConsoleMode((HANDLE)h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
			}
		}
	}
#endif
}

QString XemaCameraWindow::levelTag(XemaLogLevel level)
{
	switch (level)
	{
	case XemaLogLevel::Warn:  return "[WARN]";
	case XemaLogLevel::Error: return "[ERROR]";
	case XemaLogLevel::Exec:  return "[EXEC]";
	case XemaLogLevel::Info:
	case XemaLogLevel::Success:
	default:                  return "[INFO]"; // Success is informational -- color carries the distinction, not the tag
	}
}

// Colors by explicit level now, not by grepping the message text for Chinese/English
// keywords -- that guesswork is gone, and it means color no longer breaks if wording changes.
// highlight=true adds bold on top of the level color, for lines worth the eye even inside a
// wall of raw exe output (see logCalibExeOutput()).
QString XemaCameraWindow::colorizeForConsole(XemaLogLevel level, const QString& tagged_msg, bool highlight)
{
	const char* color = "\x1b[37m"; // default: light grey (Info)

	switch (level)
	{
	case XemaLogLevel::Error:   color = "\x1b[91m"; break; // red
	case XemaLogLevel::Warn:    color = "\x1b[93m"; break; // yellow
	case XemaLogLevel::Success: color = "\x1b[92m"; break; // green
	case XemaLogLevel::Exec:    color = "\x1b[96m"; break; // cyan
	default: break;
	}

	QString bold = highlight ? "\x1b[1m" : "";
	return bold + QString(color) + tagged_msg + "\x1b[0m";
}

// Console-only, thread-safe (WriteFile to a console handle doesn't touch any Qt widget) --
// this is what background threads call directly for live, English, leveled progress (exe
// invocations, raw subprocess stdout, disconnect/reconnect steps) that would flood the GUI's
// one-line status if routed through log() instead.
void XemaCameraWindow::logConsoleOnly(XemaLogLevel level, const QString& console_text, bool highlight)
{
#ifdef _WIN32
	if (!console_handle_) return;

	if (console_spinner_dirty_)
	{
		DWORD dummy = 0;
		WriteFile((HANDLE)console_handle_, "\r\n", 2, &dummy, nullptr);
		console_spinner_dirty_ = false;
	}

	QString tagged = levelTag(level) + " " + console_text;
	QString colored = colorizeForConsole(level, tagged, highlight);
	// toLocal8Bit on pure-ASCII English text also sidesteps any console-codepage mojibake
	// that u8"..." Chinese text was prone to on a non-UTF-8 Windows console.
	QByteArray line = colored.toLocal8Bit() + "\r\n";
	DWORD written = 0;
	WriteFile((HANDLE)console_handle_, line.constData(), (DWORD)line.size(), &written, nullptr);
#endif
}

// GUI status bar (single line, elided) + console -- must run on the GUI thread (touches
// log_view_). Only called for the final result of a user action; everything in between was
// already streamed to the console live via logConsoleOnly(). gui_text and console_text are
// independent strings -- gui_text stays short Chinese, console_text is the English equivalent.
void XemaCameraWindow::log(XemaLogLevel level, const QString& gui_text, const QString& console_text)
{
	QString single_line = gui_text;
	single_line.replace('\n', " | ");

	QFontMetrics fm(log_view_->font());
	int w = log_view_->width() > 100 ? log_view_->width() - 16 : 760;
	log_view_->setText(fm.elidedText(single_line, Qt::ElideRight, w));

	logConsoleOnly(level, console_text);
}

// ==================== busy heartbeat ====================

void XemaCameraWindow::startBusyHeartbeat(const QString& prefix)
{
	busy_prefix_ = prefix; // bracketed tag, e.g. "[calib] working" -- matches ScanToolWindow's convention
	busy_spinner_index_ = 0;
	busy_heartbeat_timer_.start(120);
}

void XemaCameraWindow::stopBusyHeartbeat()
{
	busy_heartbeat_timer_.stop();
}

// Matches ScanToolWindow::updateBusyIndicator() exactly: console gets a \r-overwritten |/-\
// spinner, and log_view_ (our terminal-styled status box, playing the same role as
// ScanToolWindow's label_status_) gets a single block character bouncing across a 12-char
// '.'-filled track -- "[calib] working  [..#.......]" -- overwritten live each tick.
void XemaCameraWindow::onBusyHeartbeat()
{
	static const QChar frames[4] = { '|', '/', '-', '\\' };
	QChar frame = frames[busy_spinner_index_ % 4];

#ifdef _WIN32
	if (console_handle_)
	{
		QString line = QString("\r\x1b[96m%1 %2 \x1b[0m").arg(busy_prefix_).arg(frame);
		QByteArray bytes = line.toLocal8Bit();
		DWORD written = 0;
		WriteFile((HANDLE)console_handle_, bytes.constData(), (DWORD)bytes.size(), &written, nullptr);
		console_spinner_dirty_ = true;
	}
#endif

	const int width = 12;
	int cycle = width * 2 - 2;
	int pos = busy_spinner_index_ % cycle;
	if (pos >= width)
	{
		pos = cycle - pos;
	}

	QString bar;
	for (int i = 0; i < width; i++)
	{
		bar += (i == pos) ? QChar(0x2588) : QChar('.');
	}
	log_view_->setText(QString("%1  [%2]").arg(busy_prefix_, bar));

	busy_spinner_index_++;
}

// ==================== connection state ====================

void XemaCameraWindow::setConnectedUiState(bool connected)
{
	connected_ = connected;
	edit_ip_->setEnabled(!connected);
	btn_connect_->setEnabled(!connected);
	btn_disconnect_->setEnabled(connected);
	btn_apply_params_->setEnabled(connected);
	btn_capture_->setEnabled(connected);
	btn_capture_calib_->setEnabled(connected);
	btn_write_params_->setEnabled(connected);
}

// ==================== connect ====================

void XemaCameraWindow::onConnectClicked()
{
	if (busy_)
	{
		log(XemaLogLevel::Warn, u8"操作进行中，请稍候", "An operation is already in progress");
		return;
	}

	QString ip = edit_ip_->text().trimmed();
	if (ip.isEmpty())
	{
		log(XemaLogLevel::Warn, u8"请输入相机IP", "Camera IP is required");
		return;
	}

	log(XemaLogLevel::Info, QString(u8"连接中: %1").arg(ip), QString("Connecting to %1...").arg(ip));
	busy_ = true;
	btn_connect_->setEnabled(false);

	std::thread t(&XemaCameraWindow::connectThreadFunc, this, ip);
	t.detach();
}

void XemaCameraWindow::connectThreadFunc(QString ip)
{
	std::string std_ip = ip.toStdString();
	int ret = DfConnect(std_ip.c_str());

	if (ret != DF_SUCCESS)
	{
		emit connectFinished(false,
			QString(u8"连接失败(%1)").arg(ret),
			QString("Connect failed: DfConnect returned error code %1").arg(ret));
		return;
	}

	DfGetCameraResolution(&width_, &height_);

	// Real exposure bounds depend on which projector is attached -- confirmed from
	// camera_capture_gui.cpp's setCameraConfigParam(): 3010 -> [1700, 100000], 4710 ->
	// [1700, 28000]. applyExposureRangeForProjector() (called from onConnectFinished, on the
	// GUI thread) uses this to narrow spin_exposure_'s range to match.
	DfGetProjectorVersion(projector_version_);

	// Determines whether DfGetBrightnessData's output is clean grayscale (Mono sensor) or
	// needs de-mosaicing internally (BayerRG8 sensor) -- diagnostic only, logged below.
	int pixel_type_ret = DfGetCameraPixelType(pixel_type_);
	if (pixel_type_ret != DF_SUCCESS)
	{
		pixel_type_ = (int)XemaPixelType::Mono;
	}

	char fw_buf[64] = { 0 };
	int fw_ret = DfGetFirmwareVersion(fw_buf);
	QString fw = (fw_ret == DF_SUCCESS) ? QString::fromUtf8(fw_buf) : QString("unknown");

	QString sensor_desc_en = (pixel_type_ == (int)XemaPixelType::BayerRG8) ? "Bayer color" : "Mono";

	// Locked to Black -- confirmed working against the real vendor GUI's own Black setting.
	// No UI to change this anymore; always set explicitly right after connecting.
	int engine_ret = DfSetCaptureEngine(XemaEngine::Black);

	logConsoleOnly(XemaLogLevel::Info, QString("Resolution %1x%2, firmware %3, projector %4, sensor %5")
		.arg(width_).arg(height_).arg(fw).arg(projector_version_).arg(sensor_desc_en));
	logConsoleOnly(engine_ret == DF_SUCCESS ? XemaLogLevel::Info : XemaLogLevel::Warn,
		QString("Capture engine set to Black (return code %1)").arg(engine_ret));

	emit connectFinished(true,
		QString(u8"已连接 %1x%2").arg(width_).arg(height_),
		QString("Connected to %1 (%2x%3, firmware %4, projector %5)")
			.arg(ip).arg(width_).arg(height_).arg(fw).arg(projector_version_));
}

void XemaCameraWindow::applyExposureRangeForProjector()
{
	double min_exposure = 1700.0, max_exposure = 100000.0; // 3010 default/fallback

	if (projector_version_ == 4710)
	{
		max_exposure = 28000.0;
	}
	else if (projector_version_ != 3010)
	{
		log(XemaLogLevel::Warn,
			QString(u8"光机型号未知(%1)").arg(projector_version_),
			QString("Unknown projector model (%1); using default exposure range [1700, 100000]").arg(projector_version_));
	}

	spin_exposure_->setRange(min_exposure, max_exposure); // Qt clamps the current value automatically if it falls outside
}

void XemaCameraWindow::logCurrentParamsInto(const QString& context)
{
	// Reads straight from the camera via DfGetParamLedCurrent/DfGetParamCameraExposure/
	// DfGetParamCameraGain -- fresh queries, not just echoing back what we last sent, so if a
	// Set call silently didn't take effect this readback shows the camera's real current value.
	int led_rb = 0;
	float exposure_rb = 0.0f, gain_rb = 0.0f;
	int ret_led = DfGetParamLedCurrent(led_rb);
	int ret_exposure = DfGetParamCameraExposure(exposure_rb);
	int ret_gain = DfGetParamCameraGain(gain_rb);

	logConsoleOnly(XemaLogLevel::Info, QString("[%1] Camera readback -- LED:%2  gain:%3  exposure:%4")
		.arg(context).arg(led_rb).arg(gain_rb).arg(exposure_rb));

	if (ret_led != DF_SUCCESS)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("[%1] Failed to read LED current (error code %2)").arg(context).arg(ret_led));
	}
	if (ret_exposure != DF_SUCCESS)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("[%1] Failed to read exposure (error code %2)").arg(context).arg(ret_exposure));
	}
	if (ret_gain != DF_SUCCESS)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("[%1] Failed to read gain (error code %2)").arg(context).arg(ret_gain));
	}
}

// ==================== calibration.exe output filtering ====================

// Full raw dump first (scroll up in the console for line-by-line detail), then a bolded
// summary block with just the three numbers worth a glance: how many board images were read,
// how many were actually detected, and the reprojection error -- colored green / yellow / red
// against thresholds sb tuned by hand (see below), not the tool's own printed 0.1 line.
// Structural lines ("Start Read Board Images......", "Start Find Board......") and the
// redundant "Calibrate Finished!" (we already report our own pass/fail line) are dropped from
// the summary -- they're still in the full dump above if needed.
void XemaCameraWindow::logCalibExeOutput(const QString& raw_output)
{
	if (raw_output.isEmpty())
	{
		return;
	}

	logConsoleOnly(XemaLogLevel::Exec, "========== [calib] output ==========", true);
	logConsoleOnly(XemaLogLevel::Info, raw_output); // full, unfiltered, passed through as-is
	logConsoleOnly(XemaLogLevel::Exec, "========== end output ==========", true);

	// Thresholds tuned by sb (2026-08-24) against real runs -- not from a spec:
	//   < 0.1        clean pass -- green
	//   0.1 - 0.2    still worth a look but not a real problem -- yellow
	//   >= 0.2       red
	const double kGoodMargin = 0.1;
	const double kThreshold = 0.2;

	QStringList lines = raw_output.split('\n');
	QString board_images_line;
	int found_count = 0;
	bool have_reprojection = false;
	double reprojection_error = 0.0;

	for (const QString& raw_line : lines)
	{
		QString line = raw_line.trimmed();
		if (line.isEmpty())
		{
			continue;
		}

		// calibration.exe repeats a bare "found" line per detected pose -- collapse the run
		// into one count instead of echoing it N times in the summary.
		if (line.compare("found", Qt::CaseInsensitive) == 0)
		{
			found_count++;
			continue;
		}

		if (line.contains("Board Images Number", Qt::CaseInsensitive))
		{
			board_images_line = line;
			continue;
		}

		// "Reprojection Error: 0.062862" -- but NOT the "...should be less than 0.1......"
		// line, which also contains the phrase "Reprojection Error".
		if (line.contains("Reprojection Error", Qt::CaseInsensitive)
			&& !line.contains("should be less than", Qt::CaseInsensitive))
		{
			int colon = line.indexOf(':');
			if (colon >= 0)
			{
				bool parsed_ok = false;
				double value = line.mid(colon + 1).trimmed().toDouble(&parsed_ok);
				if (parsed_ok)
				{
					reprojection_error = value;
					have_reprojection = true;
				}
			}
			continue;
		}

		// Everything else (section headers, the threshold-explanation line, "Calibrate
		// Finished!") is intentionally left out of the summary -- see function comment.
	}

	logConsoleOnly(XemaLogLevel::Exec, "========== [calib] summary ==========", true);

	bool have_summary_line = false;

	if (!board_images_line.isEmpty())
	{
		logConsoleOnly(XemaLogLevel::Info, board_images_line, true);
		have_summary_line = true;
	}
	if (found_count > 0)
	{
		logConsoleOnly(XemaLogLevel::Info, QString("found (x%1)").arg(found_count), true);
		have_summary_line = true;
	}
	if (have_reprojection)
	{
		XemaLogLevel level;
		if (reprojection_error < kGoodMargin)
		{
			level = XemaLogLevel::Success; // clean pass -- green, tagged [INFO]
		}
		else if (reprojection_error < kThreshold)
		{
			level = XemaLogLevel::Warn; // worth a look but not a real problem -- yellow, tagged [WARN]
		}
		else
		{
			level = XemaLogLevel::Error; // red, tagged [ERROR]
		}

		logConsoleOnly(level, QString("Reprojection Error: %1").arg(QString::number(reprojection_error, 'g', 6)), true);
		have_summary_line = true;
	}

	if (!have_summary_line)
	{
		logConsoleOnly(XemaLogLevel::Warn, "[calib] no recognizable summary lines in output -- see full output above", true);
	}
}

// See header comment: "<N>_board.bmp" is written for every pose calibration.exe processed,
// "<N>_draw.bmp" only for poses where the board was actually found. Diffing the two gives the
// exact list of poses to recapture, since calibration.exe's own stdout never names them. This
// is logged as the last line of the "[calib] summary" block logCalibExeOutput just printed.
void XemaCameraWindow::logMissingBoardPoses(const QString& identity_folder)
{
	QDir dir(identity_folder);
	QStringList board_files = dir.entryList(QStringList() << "*_board.bmp", QDir::Files);

	if (board_files.isEmpty())
	{
		// calibration.exe didn't get far enough to write per-pose board.bmp files (older
		// version, or it failed before reaching board detection) -- nothing to check here.
		return;
	}

	std::vector<int> pose_indices;
	for (const QString& f : board_files)
	{
		int underscore = f.indexOf('_');
		if (underscore <= 0)
		{
			continue;
		}
		bool parsed_ok = false;
		int n = f.left(underscore).toInt(&parsed_ok);
		if (parsed_ok)
		{
			pose_indices.push_back(n);
		}
	}
	std::sort(pose_indices.begin(), pose_indices.end());

	QStringList missing;
	for (int n : pose_indices)
	{
		if (!QFile::exists(identity_folder + QString("/%1_draw.bmp").arg(n)))
		{
			missing << QString("%1").arg(n, 2, 10, QChar('0')); // zero-padded to match the pose capture folder naming
		}
	}

	if (!missing.isEmpty())
	{
		logConsoleOnly(XemaLogLevel::Warn,
			QString("[calib] board NOT detected in pose(s): %1 -- recapture these and re-run calibration").arg(missing.join(", ")),
			true);
	}
	else
	{
		logConsoleOnly(XemaLogLevel::Info,
			QString("[calib] board detected in all %1 pose(s)").arg(pose_indices.size()),
			true);
	}

	logConsoleOnly(XemaLogLevel::Exec, "========== end summary ==========", true);
}

// ==================== pose browse page ====================

// Rescans preview_identity_folder_ for numbered pose subfolders (same "00","01",... naming
// captureForCalibThreadFunc writes), lists every one found. Color reflects the MOST RECENT
// calibration run's result (grey = board not detected in "<N>_draw.bmp", or calibration.exe
// simply hasn't been run yet; black = detected), NOT whether the pose has been captured -- see
// header comment for why these are deliberately different signals. A "✓" is appended for any
// pose in recaptured_poses_ (captured since that last calibration run, so its color is stale).
// Tries to keep whatever pose was selected before the rescan; falls back to the most recent
// pose if that one's gone or nothing was selected yet.
void XemaCameraWindow::refreshPreviewPoseList()
{
	if (preview_identity_folder_.isEmpty())
	{
		return;
	}

	QDir dir(preview_identity_folder_);
	QStringList subfolders = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);

	QStringList numeric_folders;
	for (const QString& f : subfolders)
	{
		bool parsed_ok = false;
		f.toInt(&parsed_ok);
		if (parsed_ok)
		{
			numeric_folders << f;
		}
	}
	std::sort(numeric_folders.begin(), numeric_folders.end(),
		[](const QString& a, const QString& b) { return a.toInt() < b.toInt(); });

	QString previously_selected = (preview_current_row_ >= 0 && preview_current_row_ < preview_pose_labels_.size())
		? preview_pose_labels_[preview_current_row_]
		: QString();

	preview_pose_labels_ = numeric_folders;

	preview_pose_list_->blockSignals(true); // repopulating -- don't fire currentRowChanged mid-rebuild
	preview_pose_list_->clear();
	for (const QString& label : preview_pose_labels_)
	{
		// "<N>_draw.bmp" lives at the ROOT of identity_folder, N with NO zero-padding (unlike
		// the pose subfolder names themselves) -- written by calibration.exe only when the
		// board WAS detected for that pose in its most recent run.
		int pose_num = label.toInt();
		bool was_processed = QFile::exists(preview_identity_folder_ + QString("/%1_board.bmp").arg(pose_num));
		bool board_detected = QFile::exists(preview_identity_folder_ + QString("/%1_draw.bmp").arg(pose_num));
		bool failed = was_processed && !board_detected; // only grey if calibration actually checked it and it didn't pass

		QString item_text = (failed && recaptured_poses_.contains(label)) ? label + u8" ✓" : label;
		QListWidgetItem* item = new QListWidgetItem(item_text, preview_pose_list_);
		if (failed && !recaptured_poses_.contains(label))
		{
			item->setForeground(QColor(150, 150, 150));
		}
	}
	preview_pose_list_->blockSignals(false);

	int restore_row = previously_selected.isEmpty() ? -1 : preview_pose_labels_.indexOf(previously_selected);
	if (restore_row < 0 && !preview_pose_labels_.isEmpty())
	{
		restore_row = preview_pose_labels_.size() - 1; // default to the most recent pose
	}

	if (restore_row >= 0)
	{
		loadPreviewPoseAt(restore_row);
	}
	else
	{
		preview_current_row_ = -1;
		preview_current_image_ = QImage();
		preview_window_image_label_->setPixmap(QPixmap());
		preview_window_image_label_->setText(u8"（未找到姿态照片）");
	}
}

// Loads phase36.bmp -- the confirmed preview frame index, from main_xema_color.py's own
// loadImage() (the original Python tool). Lives inside the pose's own zero-padded subfolder
// (identity_folder/<pose>/phase36.bmp), so it's available as soon as a pose is captured,
// unlike calibration.exe's <N>_draw.bmp/<N>_board.bmp diagnostic images which only exist after
// a calibration run finishes. Shows the grey "not found" placeholder if it's missing (capture
// never finished, or this pose was never actually taken).
void XemaCameraWindow::loadPreviewPoseAt(int row)
{
	if (row < 0 || row >= preview_pose_labels_.size())
	{
		return;
	}

	preview_current_row_ = row;
	QString pose_label = preview_pose_labels_[row];
	QString image_path = preview_identity_folder_ + "/" + pose_label + "/phase36.bmp";

	cv::Mat gray = cv::imread(image_path.toStdString(), cv::IMREAD_GRAYSCALE);
	if (!gray.empty())
	{
		preview_current_image_ = grayToQImage(gray);
	}
	else
	{
		preview_current_image_ = QImage();
	}

	if (!preview_current_image_.isNull())
	{
		preview_window_image_label_->setPixmap(QPixmap::fromImage(preview_current_image_).scaled(
			preview_window_image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
	}
	else
	{
		// grey "not found" placeholder -- same look as the rest of the tool's missing-preview state
		preview_window_image_label_->setPixmap(QPixmap());
		preview_window_image_label_->setText(u8"（未找到预览图）");
	}

	preview_pose_list_->blockSignals(true);
	preview_pose_list_->setCurrentRow(row);
	preview_pose_list_->blockSignals(false);
}

void XemaCameraWindow::onPreviewPoseListRowChanged(int row)
{
	if (row >= 0)
	{
		loadPreviewPoseAt(row);
	}
}

void XemaCameraWindow::onPreviewPrevClicked()
{
	if (preview_current_row_ > 0)
	{
		loadPreviewPoseAt(preview_current_row_ - 1);
	}
}

void XemaCameraWindow::onPreviewNextClicked()
{
	if (preview_current_row_ >= 0 && preview_current_row_ < preview_pose_labels_.size() - 1)
	{
		loadPreviewPoseAt(preview_current_row_ + 1);
	}
}

void XemaCameraWindow::onPreviewRefreshClicked()
{
	refreshPreviewPoseList();
}

// "浏览已拍照片" -- toggles preview_stack_ between the live feed (page 0) and the pose
// browser (page 1). Switching TO the browse page always rescans first, so it reflects
// whatever's actually on disk even if nothing was captured this session, or captures happened
// while the live page was showing (see refreshPreviewPoseListIfVisible()).
void XemaCameraWindow::onBrowsePosesClicked()
{
	if (preview_stack_->currentIndex() == 0)
	{
		preview_identity_folder_ = currentIdentityFolder();
		refreshPreviewPoseList();
		preview_stack_->setCurrentIndex(1);
		btn_browse_poses_->setText(u8"返回实时预览");
	}
	else
	{
		preview_stack_->setCurrentIndex(0);
		btn_browse_poses_->setText(u8"浏览已拍照片");
	}
}

// Called after each pose capture -- keeps the pose list's data current in the background
// without yanking the view away from whatever the person is actually looking at. Only actually
// rescans if the browse page happens to already be showing (the live page is far more likely
// mid-capture); otherwise the eventual manual toggle in onBrowsePosesClicked rescans anyway.
void XemaCameraWindow::refreshPreviewPoseListIfVisible()
{
	if (preview_stack_->currentIndex() == 1)
	{
		preview_identity_folder_ = currentIdentityFolder();
		refreshPreviewPoseList();
	}
}

// See header comment: rescales preview_current_image_ to fill preview_window_image_label_
// whenever it's resized, and steps to the previous/next pose on mouse wheel while the cursor
// is over the image (scroll up = previous, scroll down = next, matching the Prev/Next buttons).
bool XemaCameraWindow::eventFilter(QObject* watched, QEvent* event)
{
	if (watched == preview_window_image_label_)
	{
		if (event->type() == QEvent::Resize && !preview_current_image_.isNull())
		{
			preview_window_image_label_->setPixmap(QPixmap::fromImage(preview_current_image_).scaled(
				preview_window_image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
		}
		else if (event->type() == QEvent::Wheel)
		{
			QWheelEvent* wheel_event = static_cast<QWheelEvent*>(event);
			if (wheel_event->angleDelta().y() > 0)
			{
				onPreviewPrevClicked(); // scroll up -- previous pose
			}
			else if (wheel_event->angleDelta().y() < 0)
			{
				onPreviewNextClicked(); // scroll down -- next pose
			}
			return true; // consume -- there's nothing else on this label that should scroll
		}
	}
	return QWidget::eventFilter(watched, event);
}

void XemaCameraWindow::onConnectFinished(bool ok, QString guiMsg, QString consoleMsg)
{
	busy_ = false;
	log(ok ? XemaLogLevel::Success : XemaLogLevel::Error, guiMsg, consoleMsg);

	if (ok)
	{
		setConnectedUiState(true);
		applyExposureRangeForProjector();
		label_image_->setText(u8"（已连接，尚未采集）");
		label_firmware_->setText(u8"固件版本: 已连接");
		logCurrentParamsInto("post-connect");
		saveConfig();
	}
	else
	{
		btn_connect_->setEnabled(true);
	}
}

// ==================== disconnect ====================

void XemaCameraWindow::onDisconnectClicked()
{
	if (!connected_) return;

	btn_disconnect_->setEnabled(false);
	btn_capture_->setEnabled(false);
	btn_apply_params_->setEnabled(false);

	QString ip = edit_ip_->text().trimmed();
	std::thread t(&XemaCameraWindow::disconnectThreadFunc, this, ip);
	t.detach();
}

void XemaCameraWindow::disconnectThreadFunc(QString ip)
{
	bool warned = false;

	if (capturing_)
	{
		logConsoleOnly(XemaLogLevel::Info, "Stopping continuous capture...");
		capturing_ = false;

		QElapsedTimer wait_timer;
		wait_timer.start();
		while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if (capture_thread_active_)
		{
			logConsoleOnly(XemaLogLevel::Warn,
				QString("Timed out waiting for capture to stop (%1 ms); disconnecting anyway").arg(kCaptureStopTimeoutMs));
			warned = true;
		}
	}

	int ret = DfDisconnect(ip.toStdString().c_str());
	if (ret != DF_SUCCESS)
	{
		warned = true;
	}

	QString gui_msg = (ret == DF_SUCCESS) ? u8"已断开连接" : QString(u8"断开出错(%1)").arg(ret);
	QString console_msg = (ret == DF_SUCCESS) ? "Disconnected" : QString("Disconnect returned error code %1").arg(ret);

	emit disconnectFinished(gui_msg, console_msg, warned);
}

void XemaCameraWindow::onDisconnectFinished(QString guiMsg, QString consoleMsg, bool warned)
{
	setConnectedUiState(false);
	btn_capture_->setText(u8"开始连续采集");
	label_firmware_->setText(u8"固件版本: -");
	label_image_->setText(u8"（未连接）");

	log(warned ? XemaLogLevel::Warn : XemaLogLevel::Info, guiMsg, consoleMsg);
}

// ==================== params ====================

void XemaCameraWindow::onApplyParamsClicked()
{
	if (!connected_) return;

	if (applying_params_)
	{
		log(XemaLogLevel::Warn, u8"参数应用中，请稍候", "Parameter apply already in progress");
		return;
	}

	int led = spin_led_->value();
	float gain = (float)spin_gain_->value();
	float exposure = (float)spin_exposure_->value();

	applying_params_ = true;
	btn_apply_params_->setEnabled(false);
	btn_capture_->setEnabled(false); // don't let the user toggle capture while we're mid stop/resume

	std::thread t(&XemaCameraWindow::applyParamsThreadFunc, this, led, gain, exposure);
	t.detach();
}

void XemaCameraWindow::applyParamsThreadFunc(int led, float gain, float exposure)
{
	bool was_capturing = capturing_;
	bool warned = false;

	if (was_capturing)
	{
		logConsoleOnly(XemaLogLevel::Info, "Pausing continuous capture to apply parameters...");
		capturing_ = false; // request stop

		QElapsedTimer wait_timer;
		wait_timer.start();
		while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if (capture_thread_active_)
		{
			logConsoleOnly(XemaLogLevel::Warn,
				QString("Timed out waiting for capture to stop (%1 ms); applying parameters anyway").arg(kCaptureStopTimeoutMs));
			warned = true;
		}
	}

	logConsoleOnly(XemaLogLevel::Info, QString("Sending parameters -- LED:%1  gain:%2  exposure:%3").arg(led).arg(gain).arg(exposure));

	int ret_led = DfSetParamLedCurrent(led);
	int ret_exposure = DfSetParamCameraExposure(exposure);
	int ret_gain = DfSetParamCameraGain(gain);

	if (ret_led != DF_SUCCESS)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("Set LED current returned error code %1").arg(ret_led));
		warned = true;
	}
	if (ret_exposure != DF_SUCCESS)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("Set exposure returned error code %1").arg(ret_exposure));
		warned = true;
	}
	if (ret_gain != DF_SUCCESS)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("Set gain returned error code %1").arg(ret_gain));
		warned = true;
	}

	logCurrentParamsInto("post-apply");

	int led_check = 0;
	float exposure_check = 0.0f, gain_check = 0.0f;
	DfGetParamLedCurrent(led_check);
	DfGetParamCameraExposure(exposure_check);
	DfGetParamCameraGain(gain_check);

	if (std::abs(exposure_check - exposure) > 1.0f)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("Exposure did not take effect -- sent:%1  readback:%2").arg(exposure).arg(exposure_check));
		warned = true;
	}
	if (std::abs(gain_check - gain) > 0.05f)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("Gain did not take effect -- sent:%1  readback:%2").arg(gain).arg(gain_check));
		warned = true;
	}
	if (led_check != led)
	{
		logConsoleOnly(XemaLogLevel::Warn, QString("LED current did not take effect -- sent:%1  readback:%2").arg(led).arg(led_check));
		warned = true;
	}

	if (was_capturing)
	{
		capturing_ = true;
		std::thread resume_t(&XemaCameraWindow::captureLoopThreadFunc, this);
		resume_t.detach();
		logConsoleOnly(XemaLogLevel::Info, "Continuous capture resumed");
	}

	QString gui_msg = warned ? u8"参数已应用（有警告）" : u8"参数已应用";
	QString console_msg = warned ? "Parameters applied with warnings -- see detail above" : "Parameters applied successfully";

	emit applyParamsFinished(gui_msg, console_msg, warned);
}

void XemaCameraWindow::onApplyParamsFinished(QString guiMsg, QString consoleMsg, bool warned)
{
	log(warned ? XemaLogLevel::Warn : XemaLogLevel::Success, guiMsg, consoleMsg);
	applying_params_ = false;
	btn_apply_params_->setEnabled(connected_);
	btn_capture_->setEnabled(connected_);
	btn_capture_->setText(capturing_ ? u8"停止采集" : u8"开始连续采集");
	saveConfig();
}

// ==================== capture ====================

QImage XemaCameraWindow::grayToQImage(const cv::Mat& gray)
{
	return QImage(gray.data, gray.cols, gray.rows, (int)gray.step, QImage::Format_Grayscale8).copy();
}

QImage XemaCameraWindow::colorToQImage(const cv::Mat& bgr)
{
	cv::Mat rgb;
	cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
	return QImage(rgb.data, rgb.cols, rgb.rows, (int)rgb.step, QImage::Format_RGB888).copy();
}

// See header comment: always applies both overlays, no toggle. 254 (not 255) matches the
// firmware's own over-exposure threshold. Board detection uses calibration.exe's own algorithm
// -- see header comment for the three independent source confirmations.
QImage XemaCameraWindow::annotateLiveFrame(const cv::Mat& gray)
{
	cv::Mat bgr;
	cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);

	cv::Mat over_exposure_mask;
	cv::threshold(gray, over_exposure_mask, 254, 255, cv::THRESH_BINARY);
	bgr.setTo(cv::Scalar(0, 0, 255), over_exposure_mask); // BGR red

	cv::Mat inverted;
	cv::bitwise_not(gray, inverted);

	static const cv::Size kBoardSize(7, 11); // cols x rows -- constant across all board spacings
	std::vector<cv::Point2f> points;
	bool found = cv::findCirclesGrid(inverted, kBoardSize, points,
		cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);

	if (found)
	{
		cv::drawChessboardCorners(bgr, kBoardSize, cv::Mat(points), found); // works for circle grids too, not just chessboards
	}

	return colorToQImage(bgr);
}

void XemaCameraWindow::onCaptureToggled()
{
	if (!connected_)
	{
		log(XemaLogLevel::Warn, u8"请先连接相机", "Connect to the camera first");
		return;
	}

	if (!capturing_)
	{
		log(XemaLogLevel::Info, u8"开始连续采集", "Continuous capture started");
		capturing_ = true;
		btn_capture_->setText(u8"停止采集");

		std::thread t(&XemaCameraWindow::captureLoopThreadFunc, this);
		t.detach();
	}
	else
	{
		log(XemaLogLevel::Info, u8"停止连续采集中", "Stopping continuous capture...");
		capturing_ = false; // loop thread notices between frames and exits on its own
		btn_capture_->setText(u8"开始连续采集");
	}
}

void XemaCameraWindow::captureLoopThreadFunc()
{
	capture_thread_active_ = true;

	while (capturing_)
	{
		if (width_ <= 0 || height_ <= 0)
		{
			emit captureFinished(false, u8"分辨率未知", "Unknown resolution -- not connected?", QImage());
			capturing_ = false;
			break;
		}

		// DfCaptureData triggers the real structured-light capture (projector fires the
		// actual pattern sequence, firmware reconstructs depth+brightness from it) --
		// exposure_num=1 matches the real GUI's non-HDR continuous-capture path.
		char timestamp[64] = { 0 };
		int ret = DfCaptureData(1, timestamp);

		if (!capturing_)
		{
			break; // stop was requested while this capture was in flight -- discard the result
		}

		if (ret != DF_SUCCESS)
		{
			emit captureFinished(false, QString(u8"采集失败(%1)").arg(ret),
				QString("Capture failed: DfCaptureData returned error code %1").arg(ret), QImage());
			continue; // keep looping -- a single failed frame shouldn't kill continuous capture
		}

		cv::Mat gray(height_, width_, CV_8UC1, cv::Scalar(0));
		int brightness_ret = DfGetBrightnessData(gray.data);

		if (brightness_ret != DF_SUCCESS)
		{
			emit captureFinished(false, QString(u8"取图失败(%1)").arg(brightness_ret),
				QString("Failed to read brightness data: error code %1").arg(brightness_ret), QImage());
			continue;
		}

		emit captureFinished(true, u8"采集完成", "Frame captured", annotateLiveFrame(gray));
	}

	capture_thread_active_ = false;
}

void XemaCameraWindow::onCaptureFinished(bool ok, QString guiMsg, QString consoleMsg, QImage image)
{
	// During continuous capture, logging every single successful frame would flood the log
	// view -- only failures get a log line. The image still updates every frame regardless.
	if (!ok)
	{
		log(XemaLogLevel::Error, guiMsg, consoleMsg);
	}

	if (ok && !image.isNull())
	{
		label_image_->setPixmap(QPixmap::fromImage(image).scaled(
			label_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
	}
}

// ==================== capture for calib ====================

void XemaCameraWindow::onCaptureForCalibClicked()
{
	if (!connected_)
	{
		log(XemaLogLevel::Warn, u8"请先连接相机", "Connect to the camera first");
		return;
	}

	if (calib_capturing_)
	{
		log(XemaLogLevel::Warn, u8"标定拍照进行中，请稍候", "Calibration pose capture already in progress");
		return;
	}

	QString ip = edit_ip_->text().trimmed();
	if (ip.isEmpty())
	{
		log(XemaLogLevel::Warn, u8"请输入相机IP", "Camera IP is required");
		return;
	}

	// Capture GUI state on the GUI thread before spawning -- QWidget reads aren't safe from
	// a background thread. Item text IS the zero-padded pose number (see combo_start_pose_
	// header comment) -- NOT "pose_NN" -- confirmed from main_xema_color.py: calibration.exe
	// scans bare-numbered subfolders directly under the identity path it's given.
	QString pose_label = combo_start_pose_->currentText();
	QString save_folder = currentIdentityFolder() + "/" + pose_label;
	int led = spin_led_->value();
	float gain = (float)spin_gain_->value();
	float exposure = (float)spin_exposure_->value();

	calib_capturing_ = true;
	btn_capture_calib_->setEnabled(false);
	btn_capture_->setEnabled(false); // don't let continuous capture toggle while we're mid stop/capture
	btn_connect_->setEnabled(false);
	btn_disconnect_->setEnabled(false);

	std::thread t(&XemaCameraWindow::captureForCalibThreadFunc, this, ip, save_folder, pose_label, led, gain, exposure);
	t.detach();
}

int XemaCameraWindow::runExeBlocking(const QString& program, const QStringList& args, QString& out_output, int timeout_ms)
{
	QProcess proc;
	proc.setProcessChannelMode(QProcess::MergedChannels);
	proc.setWorkingDirectory(QCoreApplication::applicationDirPath());
	proc.start(program, args);

	if (!proc.waitForStarted(5000))
	{
		out_output = QString("Failed to start %1 -- check that it and its DLLs are next to this program").arg(program);
		return -1;
	}

	if (!proc.waitForFinished(timeout_ms))
	{
		proc.kill();
		proc.waitForFinished(2000);
		out_output = QString("Timed out after %1 ms -- process was killed").arg(timeout_ms);
		return -2;
	}

	out_output = QString::fromLocal8Bit(proc.readAllStandardOutput()).trimmed();
	return proc.exitCode();
}

void XemaCameraWindow::captureForCalibThreadFunc(QString ip, QString save_folder, QString pose_label, int led, float gain, float exposure)
{
	bool was_capturing = capturing_;

	if (was_capturing)
	{
		logConsoleOnly(XemaLogLevel::Info, "Stopping continuous capture for calibration pose capture...");
		capturing_ = false; // request stop

		QElapsedTimer wait_timer;
		wait_timer.start();
		while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if (capture_thread_active_)
		{
			logConsoleOnly(XemaLogLevel::Warn,
				QString("Timed out waiting for capture to stop (%1 ms); proceeding with pose capture anyway").arg(kCaptureStopTimeoutMs));
		}
	}

	// NOTE: --get-raw-02's own server-side handler never touches exposure/gain/LED at all --
	// it just fires a fixed projector pattern sequence and grabs raw frames. These Set calls
	// don't affect what --get-raw-02 captures; kept only because they're still meaningful for
	// whatever state the camera is in before/after this disconnect.
	logConsoleOnly(XemaLogLevel::Info,
		QString("Pre-capture parameters -- LED:%1  gain:%2  exposure:%3 (note: --get-raw-02 does not use these)").arg(led).arg(gain).arg(exposure));
	DfSetParamLedCurrent(led);
	DfSetParamCameraExposure(exposure);
	DfSetParamCameraGain(gain);

	logCurrentParamsInto("pre-calib-capture, before disconnect");

	// --get-raw-02 opens its OWN connection to the camera -- the camera very likely only
	// accepts one client at a time, so our in-process DfConnect session has to step aside first.
	logConsoleOnly(XemaLogLevel::Info, "Disconnecting in-process session so the external tool can capture...");
	DfDisconnect(ip.toStdString().c_str());
	connected_ = false;

	QDir().mkpath(save_folder);

	QStringList args;
	args << "--get-raw-02" << "--ip" << ip << "--path" << QDir::toNativeSeparators(save_folder);

	logConsoleOnly(XemaLogLevel::Exec, QString("[pose-capture] running: open_cam3d.exe %1").arg(args.join(' ')));

	QString exe_output;
	int exit_code = runExeBlocking("open_cam3d.exe", args, exe_output, 60000);
	if (!exe_output.isEmpty())
	{
		logConsoleOnly(XemaLogLevel::Exec, "========== [pose-capture] output ==========", true);
		logConsoleOnly(XemaLogLevel::Info, exe_output); // raw subprocess stdout -- passed through as-is
		logConsoleOnly(XemaLogLevel::Exec, "========== end ==========", true);
	}
	logConsoleOnly(exit_code == 0 ? XemaLogLevel::Info : XemaLogLevel::Warn,
		QString("[pose-capture] finished (exit code %1)").arg(exit_code));

	logConsoleOnly(XemaLogLevel::Info, "Reconnecting...");
	int reconnect_ret = DfConnect(ip.toStdString().c_str());

	QImage out_image;
	QString gui_msg;
	QString console_msg;
	bool ok = false;

	if (reconnect_ret == DF_SUCCESS)
	{
		connected_ = true;
		DfSetCaptureEngine(XemaEngine::Black);
		DfSetParamLedCurrent(led);
		DfSetParamCameraExposure(exposure);
		DfSetParamCameraGain(gain);
		logConsoleOnly(XemaLogLevel::Info, "Reconnected -- Black engine and current parameters restored");

		QString preview_path = save_folder + "/phase36.bmp";
		cv::Mat gray = cv::imread(preview_path.toStdString(), cv::IMREAD_GRAYSCALE);

		if (!gray.empty())
		{
			out_image = grayToQImage(gray);

			if (exit_code == 0)
			{
				ok = true;
			}

			gui_msg = ok ? QString(u8"标定拍照完成，姿态已保存") : QString(u8"标定拍照完成但退出码异常(%1)").arg(exit_code);
			console_msg = QString("Calibration pose saved to %1").arg(save_folder);
		}
		else
		{
			gui_msg = u8"标定拍照失败，未读取到预览图";
			console_msg = QString("Failed to read preview image %1 -- capture may not have succeeded").arg(preview_path);
		}
	}
	else
	{
		gui_msg = QString(u8"重连失败(%1)，请手动连接").arg(reconnect_ret);
		console_msg = QString("Reconnect failed: error code %1 -- please connect manually").arg(reconnect_ret);
	}

	// Auto-resumes continuous capture afterward if it was running before.
	if (was_capturing && connected_)
	{
		capturing_ = true;
		std::thread resume_t(&XemaCameraWindow::captureLoopThreadFunc, this);
		resume_t.detach();
		logConsoleOnly(XemaLogLevel::Info, "Continuous capture automatically resumed");
	}

	emit calibCaptureFinished(ok, gui_msg, console_msg, out_image, pose_label);
}

void XemaCameraWindow::onCalibCaptureFinished(bool ok, QString guiMsg, QString consoleMsg, QImage image, QString poseLabel)
{
	log(ok ? XemaLogLevel::Success : XemaLogLevel::Error, guiMsg, consoleMsg);

	if (!image.isNull())
	{
		label_image_->setPixmap(QPixmap::fromImage(image).scaled(
			label_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

		// A real photo landed for this pose, regardless of the exe's own exit code -- its
		// previous grey/black verdict (if any) is now stale until the next calibration run.
		recaptured_poses_.insert(poseLabel);

		// Same mechanism as ScanToolWindow::onShotClicked: advance to the next slot after a
		// successful capture, capped at the last item -- doesn't move if the user deliberately
		// picked an earlier number to recapture/overwrite a specific pose.
		if (combo_start_pose_->currentIndex() < combo_start_pose_->count() - 1)
		{
			combo_start_pose_->setCurrentIndex(combo_start_pose_->currentIndex() + 1);
		}

		refreshPreviewPoseListIfVisible(); // keeps the pose list current in the background -- see header comment on why this doesn't auto-switch pages
	}

	calib_capturing_ = false;
	setConnectedUiState(connected_);
	if (connected_)
	{
		label_firmware_->setText(u8"固件版本: 已连接");
	}
	btn_capture_->setText(capturing_ ? u8"停止采集" : u8"开始连续采集");
}

// ==================== calibrate ====================

void XemaCameraWindow::onCalibrateClicked()
{
	if (calibrating_)
	{
		log(XemaLogLevel::Warn, u8"标定进行中，请稍候", "Calibration already in progress");
		return;
	}

	QString identity_folder = currentIdentityFolder();

	if (!QDir(identity_folder).exists())
	{
		log(XemaLogLevel::Warn,
			QString(u8"未找到姿态目录: %1").arg(identity_folder),
			QString("Pose folder not found: %1 -- capture at least a few poses first").arg(identity_folder));
		return;
	}

	// projector_version_ is only ever set by a real DfGetProjectorVersion() at connect time (or
	// loaded from a PREVIOUS session's successful connect, via loadConfig()) -- 3010/4710 are
	// the only real models. If it's still 0 (never connected, ever, on this machine),
	// calibration.exe would get an invalid --version 0. Calibration is designed to run fully
	// offline on purpose, so this doesn't block -- just makes sure the reason for a likely-bad
	// result is visible in the log instead of a silent wrong --version.
	if (projector_version_ != 3010 && projector_version_ != 4710)
	{
		log(XemaLogLevel::Warn,
			u8"光机型号未知，标定可能失败",
			QString("Projector model unknown (currently %1) -- connect to the camera at least once (even briefly, then disconnect) so the correct --version is known and saved for future offline calibration runs").arg(projector_version_));
	}

	calibrating_ = true;
	btn_calibrate_->setEnabled(false);
	logConsoleOnly(XemaLogLevel::Info, "[calib] starting");
	startBusyHeartbeat("[calib] working");

	std::thread t(&XemaCameraWindow::calibrateThreadFunc, this, identity_folder, projector_version_, (int)board_spacing_);
	t.detach();
}

void XemaCameraWindow::calibrateThreadFunc(QString identity_folder, int projector_version, int board_spacing_mm)
{
	// Matches main_xema_color.py's calibrate() exactly:
	//   calibration.exe --calibrate --use patterns-c --path <identity>/ --version <projector>
	//     --board <spacing> --calib <identity>/param.txt
	QString calib_path = identity_folder + "/param.txt";

	// Delete any stale param.txt from a previous run first -- this exe's exit code isn't a
	// reliable success signal (confirmed: a run that printed "Calibrate Finished!" with a
	// reprojection error well under threshold still returned exit code 1). Freshly-written-
	// this-run existence of calib_path is the actual evidence of success.
	if (QFile::exists(calib_path))
	{
		QFile::remove(calib_path);
	}

	QStringList args;
	args << "--calibrate" << "--use" << "patterns-c"
		<< "--path" << QDir::toNativeSeparators(identity_folder + "/")
		<< "--version" << QString::number(projector_version)
		<< "--board" << QString::number(board_spacing_mm)
		<< "--calib" << QDir::toNativeSeparators(calib_path);

	logConsoleOnly(XemaLogLevel::Exec, QString("[calib] running: calibration.exe %1").arg(args.join(' ')));

	QString exe_output;
	// Calibration across many poses can genuinely take minutes -- much longer timeout than
	// the single-frame raw capture uses.
	int exit_code = runExeBlocking("calibration.exe", args, exe_output, 600000);
	logCalibExeOutput(exe_output); // full raw dump, then a bolded summary block
	logMissingBoardPoses(identity_folder); // diffs <N>_board.bmp vs <N>_draw.bmp on disk -- names exactly which poses to recapture, closes the summary block

	bool ok = QFile::exists(calib_path);
	QString gui_msg = ok ? u8"标定完成" : u8"标定失败";
	QString console_msg = ok
		? QString("[calib] finished -- %1 written").arg(calib_path)
		: QString("[calib] failed -- %1 was not created (exit code %2)").arg(calib_path).arg(exit_code);

	emit calibrateFinished(gui_msg, console_msg, ok);
}

void XemaCameraWindow::onCalibrateFinished(QString guiMsg, QString consoleMsg, bool ok)
{
	stopBusyHeartbeat();
	log(ok ? XemaLogLevel::Success : XemaLogLevel::Error, guiMsg, consoleMsg);

	calibrating_ = false;
	btn_calibrate_->setEnabled(true);

	// Every pose's grey/black verdict is now freshly known (see refreshPreviewPoseList()) --
	// any "✓" marks from captures since the last run are no longer meaningful either way.
	recaptured_poses_.clear();
	refreshPreviewPoseListIfVisible();
}

// ==================== write params ====================

void XemaCameraWindow::onWriteParamsClicked()
{
	if (!connected_)
	{
		log(XemaLogLevel::Warn, u8"请先连接相机", "Connect to the camera first");
		return;
	}

	if (writing_params_)
	{
		log(XemaLogLevel::Warn, u8"写参数进行中，请稍候", "Write-parameters already in progress");
		return;
	}

	QString ip = edit_ip_->text().trimmed();
	if (ip.isEmpty())
	{
		log(XemaLogLevel::Warn, u8"请输入相机IP", "Camera IP is required");
		return;
	}

	QString calib_path = currentIdentityFolder() + "/param.txt";
	if (!QFile::exists(calib_path))
	{
		log(XemaLogLevel::Warn,
			QString(u8"未找到 param.txt: %1").arg(calib_path),
			QString("param.txt not found at %1 -- run calibration first").arg(calib_path));
		return;
	}

	int led = spin_led_->value();
	float gain = (float)spin_gain_->value();
	float exposure = (float)spin_exposure_->value();

	writing_params_ = true;
	btn_write_params_->setEnabled(false);
	btn_capture_calib_->setEnabled(false);
	btn_capture_->setEnabled(false);
	btn_connect_->setEnabled(false);
	btn_disconnect_->setEnabled(false);
	logConsoleOnly(XemaLogLevel::Info, "[write] starting");
	startBusyHeartbeat("[write] working");

	std::thread t(&XemaCameraWindow::writeParamsThreadFunc, this, ip, calib_path, led, gain, exposure);
	t.detach();
}

void XemaCameraWindow::writeParamsThreadFunc(QString ip, QString calib_path, int led, float gain, float exposure)
{
	bool was_capturing = capturing_;

	if (was_capturing)
	{
		logConsoleOnly(XemaLogLevel::Info, "Stopping continuous capture to write parameters...");
		capturing_ = false;

		QElapsedTimer wait_timer;
		wait_timer.start();
		while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if (capture_thread_active_)
		{
			logConsoleOnly(XemaLogLevel::Warn,
				QString("Timed out waiting for capture to stop (%1 ms); writing parameters anyway").arg(kCaptureStopTimeoutMs));
		}
	}

	DfSetParamLedCurrent(led);
	DfSetParamCameraExposure(exposure);
	DfSetParamCameraGain(gain);

	logConsoleOnly(XemaLogLevel::Info, "Disconnecting in-process session so the external tool can write parameters...");
	DfDisconnect(ip.toStdString().c_str());
	connected_ = false;

	QStringList args;
	args << "--set-calib-looktable" << "--ip" << ip << "--path" << QDir::toNativeSeparators(calib_path);

	logConsoleOnly(XemaLogLevel::Exec, QString("[write] running: open_cam3d.exe %1").arg(args.join(' ')));

	QString exe_output;
	int exit_code = runExeBlocking("open_cam3d.exe", args, exe_output, 60000);
	if (!exe_output.isEmpty())
	{
		logConsoleOnly(XemaLogLevel::Exec, "========== [write] output ==========", true);
		logConsoleOnly(XemaLogLevel::Info, exe_output);
		logConsoleOnly(XemaLogLevel::Exec, "========== end ==========", true);
	}
	logConsoleOnly(exit_code == 0 ? XemaLogLevel::Info : XemaLogLevel::Warn,
		QString("[write] finished (exit code %1)").arg(exit_code));

	logConsoleOnly(XemaLogLevel::Info, "Reconnecting...");
	int reconnect_ret = DfConnect(ip.toStdString().c_str());

	bool ok = false;
	QString gui_msg;
	QString console_msg;

	if (reconnect_ret == DF_SUCCESS)
	{
		connected_ = true;
		DfSetCaptureEngine(XemaEngine::Black);
		DfSetParamLedCurrent(led);
		DfSetParamCameraExposure(exposure);
		DfSetParamCameraGain(gain);
		logConsoleOnly(XemaLogLevel::Info, "Reconnected -- Black engine and current parameters restored");

		ok = (exit_code == 0);
		gui_msg = ok ? u8"写参数完成" : QString(u8"写参数完成但退出码异常(%1)").arg(exit_code);
		console_msg = ok
			? QString("Parameters written to camera successfully")
			: QString("Write command exited with code %1 -- verify parameters on the camera").arg(exit_code);

		if (was_capturing)
		{
			capturing_ = true;
			std::thread resume_t(&XemaCameraWindow::captureLoopThreadFunc, this);
			resume_t.detach();
			logConsoleOnly(XemaLogLevel::Info, "Continuous capture automatically resumed");
		}
	}
	else
	{
		gui_msg = QString(u8"重连失败(%1)，请手动连接").arg(reconnect_ret);
		console_msg = QString("Reconnect failed: error code %1 -- please connect manually").arg(reconnect_ret);
	}

	emit writeParamsFinished(gui_msg, console_msg, ok);
}

void XemaCameraWindow::onWriteParamsFinished(QString guiMsg, QString consoleMsg, bool ok)
{
	stopBusyHeartbeat();
	log(ok ? XemaLogLevel::Success : XemaLogLevel::Error, guiMsg, consoleMsg);

	writing_params_ = false;
	setConnectedUiState(connected_);
	if (connected_)
	{
		label_firmware_->setText(u8"固件版本: 已连接");
	}
	btn_capture_->setText(capturing_ ? u8"停止采集" : u8"开始连续采集");
}