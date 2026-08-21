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
#include <thread>
#include <cmath>
#include <QElapsedTimer>
#include <QAbstractButton>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
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
	btn_capture_calib_ = new QPushButton(u8"拍照（用于标定）", this);
	btn_capture_calib_->setEnabled(false);
	btn_capture_calib_->setToolTip(u8"停止连续采集（如果正在运行），拍摄一张单独的标定用图像并保存编号");

	QHBoxLayout* capture_row = new QHBoxLayout();
	capture_row->addWidget(btn_capture_);
	capture_row->addWidget(btn_capture_calib_);

	// ---- calibrate ----
	btn_calibrate_ = new QPushButton(u8"标定", this);
	btn_calibrate_->setEnabled(true); // no camera connection needed
	btn_calibrate_->setToolTip(u8"对已拍摄的标定姿态运行 calibration.exe，不需要相机连接");
	label_calib_status_ = new QLabel(u8"标定: -", this);

	btn_write_params_ = new QPushButton(u8"写参数", this);
	btn_write_params_->setEnabled(false); // needs a live connection, like capture-for-calib
	btn_write_params_->setToolTip(u8"将标定结果 (param.txt) 写入相机 -- 需要先成功运行标定");
	label_write_status_ = new QLabel(u8"写参数: -", this);

	QHBoxLayout* calibrate_row = new QHBoxLayout();
	calibrate_row->addWidget(btn_calibrate_);
	calibrate_row->addWidget(label_calib_status_);
	calibrate_row->addWidget(btn_write_params_);
	calibrate_row->addWidget(label_write_status_);
	calibrate_row->addStretch();

	label_image_ = new QLabel(this);
	label_image_->setMinimumHeight(350);
	label_image_->setAlignment(Qt::AlignCenter);
	label_image_->setText(u8"（未连接）");
	label_image_->setStyleSheet("background-color:#222; color:#aaa; border: 1px solid #888;");

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
	main_layout->addWidget(label_image_, 1);
	main_layout->addWidget(log_view_);

	connect(btn_connect_, &QPushButton::clicked, this, &XemaCameraWindow::onConnectClicked);
	connect(btn_disconnect_, &QPushButton::clicked, this, &XemaCameraWindow::onDisconnectClicked);
	connect(btn_apply_params_, &QPushButton::clicked, this, &XemaCameraWindow::onApplyParamsClicked);
	connect(btn_capture_, &QPushButton::clicked, this, &XemaCameraWindow::onCaptureToggled);
	connect(btn_capture_calib_, &QPushButton::clicked, this, &XemaCameraWindow::onCaptureForCalibClicked);
	connect(btn_calibrate_, &QPushButton::clicked, this, &XemaCameraWindow::onCalibrateClicked);
	connect(btn_write_params_, &QPushButton::clicked, this, &XemaCameraWindow::onWriteParamsClicked);
	connect(btn_browse_save_path_, &QPushButton::clicked, this, &XemaCameraWindow::onBrowseSavePathClicked);
	connect(group_board_spacing_, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked), this, &XemaCameraWindow::onBoardSpacingChanged);
	connect(this, &XemaCameraWindow::connectFinished, this, &XemaCameraWindow::onConnectFinished);
	connect(this, &XemaCameraWindow::captureFinished, this, &XemaCameraWindow::onCaptureFinished);
	connect(this, &XemaCameraWindow::applyParamsFinished, this, &XemaCameraWindow::onApplyParamsFinished);
	connect(this, &XemaCameraWindow::disconnectFinished, this, &XemaCameraWindow::onDisconnectFinished);
	connect(this, &XemaCameraWindow::calibCaptureFinished, this, &XemaCameraWindow::onCalibCaptureFinished);
	connect(this, &XemaCameraWindow::calibrateFinished, this, &XemaCameraWindow::onCalibrateFinished);
	connect(this, &XemaCameraWindow::writeParamsFinished, this, &XemaCameraWindow::onWriteParamsFinished);
	connect(&busy_heartbeat_timer_, &QTimer::timeout, this, &XemaCameraWindow::onBusyHeartbeat);

	loadConfig();
	log(u8"就绪。");
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

	log(QString(u8"标定板间距设置为 %1mm").arg((int)board_spacing_));
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
		SetConsoleTitleA("XEMA Camera GUI - detail log");

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

QString XemaCameraWindow::colorizeForConsole(const QString& msg)
{
	QString lower = msg.toLower();
	const char* color = "\x1b[37m";

	if (lower.contains(u8"失败") || lower.contains(u8"错误") || lower.contains("fail") || lower.contains("error"))
	{
		color = "\x1b[91m"; // red
	}
	else if (lower.contains(u8"警告") || lower.contains("warning"))
	{
		color = "\x1b[93m"; // yellow
	}
	else if (lower.contains(u8"完成") || lower.contains(u8"成功") || lower.contains(u8"已恢复")
		|| lower.contains(u8"已连接") || lower.contains("success"))
	{
		color = "\x1b[92m"; // green
	}
	else if (msg.startsWith(u8"运行:") || msg.startsWith("=========="))
	{
		color = "\x1b[96m"; // cyan
	}

	return QString(color) + msg + "\x1b[0m";
}

// Console-only, thread-safe (WriteFile to a console handle doesn't touch any Qt widget) --
// use this from background threads for verbose detail (raw exe stdout, connection trace)
// that shouldn't clutter the GUI's short summary log.
void XemaCameraWindow::logConsoleOnly(const QString& msg)
{
#ifdef _WIN32
	if (!console_handle_) return;

	if (console_spinner_dirty_)
	{
		DWORD dummy = 0;
		WriteFile((HANDLE)console_handle_, "\r\n", 2, &dummy, nullptr);
		console_spinner_dirty_ = false;
	}

	QString colored = colorizeForConsole(msg);
	QByteArray line = colored.toLocal8Bit() + "\r\n";
	DWORD written = 0;
	WriteFile((HANDLE)console_handle_, line.constData(), (DWORD)line.size(), &written, nullptr);
#endif
}

// GUI status bar (single line, elided) + console -- must run on the GUI thread (touches
// log_view_), so background threads use logConsoleOnly() directly and let the eventual
// signal-driven finished-slot call this only for the short final summary. Multi-line
// messages get flattened to " | "-separated segments for the GUI line; the console still
// gets the original with real line breaks.
void XemaCameraWindow::log(const QString& msg)
{
	QString single_line = msg;
	single_line.replace('\n', " | ");

	QFontMetrics fm(log_view_->font());
	int w = log_view_->width() > 100 ? log_view_->width() - 16 : 760;
	log_view_->setText(fm.elidedText(single_line, Qt::ElideRight, w));

	logConsoleOnly(msg);
}

// ==================== busy heartbeat ====================

void XemaCameraWindow::startBusyHeartbeat(QLabel* label, const QString& prefix)
{
	busy_label_ = label;
	busy_prefix_ = prefix;
	if (busy_label_)
	{
		busy_label_->setText(prefix); // set once -- stays static, no animation in the GUI
		busy_label_->setStyleSheet("");
	}
	spinner_index_ = 0;
	busy_heartbeat_timer_.start(150);
}

void XemaCameraWindow::stopBusyHeartbeat()
{
	busy_heartbeat_timer_.stop();
	busy_label_ = nullptr;
}

// Console-only animation -- the GUI status label stays a plain static line the whole time
// (set once in startBusyHeartbeat), matching the "keep the tool simple, one line" request.
void XemaCameraWindow::onBusyHeartbeat()
{
#ifdef _WIN32
	if (console_handle_)
	{
		static const QChar frames[4] = { '|', '/', '-', '\\' };
		QChar frame = frames[spinner_index_ % 4];

		QString line = QString("\r\x1b[96m%1 %2 \x1b[0m").arg(busy_prefix_).arg(frame);
		QByteArray bytes = line.toLocal8Bit();
		DWORD written = 0;
		WriteFile((HANDLE)console_handle_, bytes.constData(), (DWORD)bytes.size(), &written, nullptr);
		console_spinner_dirty_ = true;
	}
#endif

	spinner_index_++;
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
		log(u8"操作进行中，请稍候...");
		return;
	}

	QString ip = edit_ip_->text().trimmed();
	if (ip.isEmpty())
	{
		log(u8"请输入相机IP");
		return;
	}

	log(QString(u8"正在连接 %1 ...").arg(ip));
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
		emit connectFinished(false, QString(u8"连接失败，错误码: %1").arg(ret));
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
	QString fw = (fw_ret == DF_SUCCESS) ? QString::fromUtf8(fw_buf) : QString(u8"获取失败");

	QString sensor_desc = (pixel_type_ == (int)XemaPixelType::BayerRG8) ? u8"彩色(Bayer)" : u8"黑白(Mono)";

	// Locked to Black -- confirmed working against the real vendor GUI's own Black setting.
	// No UI to change this anymore; always set explicitly right after connecting.
	int engine_ret = DfSetCaptureEngine(XemaEngine::Black);

	emit connectFinished(true, QString(u8"连接成功。分辨率: %1x%2  固件: %3  光机型号: %4  传感器: %5  采集引擎: 黑白(Black) (返回码 %6)")
		.arg(width_).arg(height_).arg(fw).arg(projector_version_).arg(sensor_desc).arg(engine_ret));
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
		log(QString(u8"未知光机型号 (%1)，曝光范围使用默认值").arg(projector_version_));
	}

	spin_exposure_->setRange(min_exposure, max_exposure); // Qt clamps the current value automatically if it falls outside
}

void XemaCameraWindow::logCurrentParamsInto(const QString& context, QStringList& lines_out)
{
	// Reads straight from the camera via DfGetParamLedCurrent/DfGetParamCameraExposure/
	// DfGetParamCameraGain -- fresh queries, not just echoing back what we last sent, so if a
	// Set call silently didn't take effect this readback shows the camera's real current value.
	int led_rb = 0;
	float exposure_rb = 0.0f, gain_rb = 0.0f;
	int ret_led = DfGetParamLedCurrent(led_rb);
	int ret_exposure = DfGetParamCameraExposure(exposure_rb);
	int ret_gain = DfGetParamCameraGain(gain_rb);

	lines_out << QString(u8"[%1] 相机当前实际值 -- LED:%2  增益:%3  曝光:%4")
		.arg(context).arg(led_rb).arg(gain_rb).arg(exposure_rb);

	if (ret_led != DF_SUCCESS)
	{
		lines_out << QString(u8"[%1] 警告: 读取LED失败，错误码 %2").arg(context).arg(ret_led);
	}
	if (ret_exposure != DF_SUCCESS)
	{
		lines_out << QString(u8"[%1] 警告: 读取曝光失败，错误码 %2").arg(context).arg(ret_exposure);
	}
	if (ret_gain != DF_SUCCESS)
	{
		lines_out << QString(u8"[%1] 警告: 读取增益失败，错误码 %2").arg(context).arg(ret_gain);
	}
}

void XemaCameraWindow::onConnectFinished(bool ok, QString message)
{
	busy_ = false;
	log(message);

	if (ok)
	{
		setConnectedUiState(true);
		applyExposureRangeForProjector();
		label_image_->setText(u8"（已连接，尚未采集）");
		label_firmware_->setText(u8"固件版本: 已连接");
		QStringList lines;
		logCurrentParamsInto(u8"连接后", lines);
		log(lines.join('\n'));
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
	QStringList lines;

	if (capturing_)
	{
		lines << u8"正在停止连续采集...";
		capturing_ = false;

		QElapsedTimer wait_timer;
		wait_timer.start();
		while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if (capture_thread_active_)
		{
			lines << QString(u8"警告: 等待采集停止超时 (%1ms)，仍继续断开").arg(kCaptureStopTimeoutMs);
		}
	}

	int ret = DfDisconnect(ip.toStdString().c_str());
	lines << (ret == DF_SUCCESS ? u8"已断开连接。" : QString(u8"断开时返回错误码: %1").arg(ret));

	emit disconnectFinished(lines.join('\n'));
}

void XemaCameraWindow::onDisconnectFinished(QString message)
{
	setConnectedUiState(false);
	btn_capture_->setText(u8"开始连续采集");
	label_firmware_->setText(u8"固件版本: -");
	label_image_->setText(u8"（未连接）");

	log(message);
}

// ==================== params ====================

void XemaCameraWindow::onApplyParamsClicked()
{
	if (!connected_) return;

	if (applying_params_)
	{
		log(u8"参数应用正在进行中，请稍候...");
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
	QStringList lines;
	bool was_capturing = capturing_;

	if (was_capturing)
	{
		lines << u8"正在暂停连续采集以应用参数...";
		capturing_ = false; // request stop

		QElapsedTimer wait_timer;
		wait_timer.start();
		while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if (capture_thread_active_)
		{
			lines << QString(u8"警告: 等待采集停止超时 (%1ms)，仍继续应用参数 -- 可能短暂卡顿").arg(kCaptureStopTimeoutMs);
		}
	}

	lines << QString(u8"发送设置 -- LED:%1  增益:%2  曝光:%3").arg(led).arg(gain).arg(exposure);

	int ret_led = DfSetParamLedCurrent(led);
	int ret_exposure = DfSetParamCameraExposure(exposure);
	int ret_gain = DfSetParamCameraGain(gain);

	if (ret_led != DF_SUCCESS)
	{
		lines << QString(u8"警告: 设置LED返回错误码 %1").arg(ret_led);
	}
	if (ret_exposure != DF_SUCCESS)
	{
		lines << QString(u8"警告: 设置曝光返回错误码 %1").arg(ret_exposure);
	}
	if (ret_gain != DF_SUCCESS)
	{
		lines << QString(u8"警告: 设置增益返回错误码 %1").arg(ret_gain);
	}

	logCurrentParamsInto(u8"应用后", lines);

	int led_check = 0;
	float exposure_check = 0.0f, gain_check = 0.0f;
	DfGetParamLedCurrent(led_check);
	DfGetParamCameraExposure(exposure_check);
	DfGetParamCameraGain(gain_check);

	if (std::abs(exposure_check - exposure) > 1.0f)
	{
		lines << QString(u8"!! 曝光设置似乎未生效 -- 发送:%1  相机返回:%2").arg(exposure).arg(exposure_check);
	}
	if (std::abs(gain_check - gain) > 0.05f)
	{
		lines << QString(u8"!! 增益设置似乎未生效 -- 发送:%1  相机返回:%2").arg(gain).arg(gain_check);
	}
	if (led_check != led)
	{
		lines << QString(u8"!! LED设置似乎未生效 -- 发送:%1  相机返回:%2").arg(led).arg(led_check);
	}

	if (was_capturing)
	{
		lines << u8"恢复连续采集...";
		capturing_ = true;
		std::thread resume_t(&XemaCameraWindow::captureLoopThreadFunc, this);
		resume_t.detach();
	}

	emit applyParamsFinished(lines.join('\n'));
}

void XemaCameraWindow::onApplyParamsFinished(QString message)
{
	log(message);
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

void XemaCameraWindow::onCaptureToggled()
{
	if (!connected_)
	{
		log(u8"请先连接相机");
		return;
	}

	if (!capturing_)
	{
		log(u8"开始连续采集");
		capturing_ = true;
		btn_capture_->setText(u8"停止采集");

		std::thread t(&XemaCameraWindow::captureLoopThreadFunc, this);
		t.detach();
	}
	else
	{
		log(u8"停止连续采集...");
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
			emit captureFinished(false, u8"分辨率未知（未连接？）", QImage());
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
			emit captureFinished(false, QString(u8"采集失败，错误码: %1").arg(ret), QImage());
			continue; // keep looping -- a single failed frame shouldn't kill continuous capture
		}

		cv::Mat gray(height_, width_, CV_8UC1, cv::Scalar(0));
		int brightness_ret = DfGetBrightnessData(gray.data);

		if (brightness_ret != DF_SUCCESS)
		{
			emit captureFinished(false, QString(u8"获取亮度图失败，错误码: %1").arg(brightness_ret), QImage());
			continue;
		}

		emit captureFinished(true, u8"采集完成。", grayToQImage(gray));
	}

	capture_thread_active_ = false;
}

void XemaCameraWindow::onCaptureFinished(bool ok, QString message, QImage image)
{
	// During continuous capture, logging every single successful frame would flood the log
	// view -- only failures get a log line. The image still updates every frame regardless.
	if (!ok)
	{
		log(message);
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
		log(u8"请先连接相机");
		return;
	}

	if (calib_capturing_)
	{
		log(u8"标定拍照正在进行中，请稍候...");
		return;
	}

	QString ip = edit_ip_->text().trimmed();
	if (ip.isEmpty())
	{
		log(u8"请输入相机IP");
		return;
	}

	// Capture GUI state on the GUI thread before spawning -- QWidget reads aren't safe from
	// a background thread. Bare zero-padded numeric folder name (00, 01, ...), NOT "pose_NN"
	// -- confirmed from main_xema_color.py: calibration.exe scans bare-numbered subfolders
	// directly under the identity path it's given.
	QString save_folder = currentIdentityFolder() + QString("/%1").arg(calib_pose_index_, 2, 10, QChar('0'));
	int led = spin_led_->value();
	float gain = (float)spin_gain_->value();
	float exposure = (float)spin_exposure_->value();

	calib_capturing_ = true;
	btn_capture_calib_->setEnabled(false);
	btn_capture_->setEnabled(false); // don't let continuous capture toggle while we're mid stop/capture
	btn_connect_->setEnabled(false);
	btn_disconnect_->setEnabled(false);

	std::thread t(&XemaCameraWindow::captureForCalibThreadFunc, this, ip, save_folder, led, gain, exposure);
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
		out_output = QString(u8"无法启动: %1 (请确认它和它的DLL在本程序所在目录)").arg(program);
		return -1;
	}

	if (!proc.waitForFinished(timeout_ms))
	{
		proc.kill();
		proc.waitForFinished(2000);
		out_output = QString(u8"超时 (%1ms)，已终止").arg(timeout_ms);
		return -2;
	}

	out_output = QString::fromLocal8Bit(proc.readAllStandardOutput()).trimmed();
	return proc.exitCode();
}

void XemaCameraWindow::captureForCalibThreadFunc(QString ip, QString save_folder, int led, float gain, float exposure)
{
	QStringList lines;
	bool was_capturing = capturing_;

	if (was_capturing)
	{
		lines << u8"正在停止连续采集以进行标定拍照...";
		capturing_ = false; // request stop

		QElapsedTimer wait_timer;
		wait_timer.start();
		while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if (capture_thread_active_)
		{
			lines << QString(u8"警告: 等待采集停止超时 (%1ms)，仍继续标定拍照 -- 可能短暂卡顿").arg(kCaptureStopTimeoutMs);
		}
	}

	// NOTE: --get-raw-02's own server-side handler never touches exposure/gain/LED at all --
	// it just fires a fixed projector pattern sequence and grabs raw frames. These Set calls
	// don't affect what --get-raw-02 captures; kept only because they're still meaningful for
	// whatever state the camera is in before/after this disconnect.
	logConsoleOnly(QString(u8"发送标定拍照前设置 -- LED:%1  增益:%2  曝光:%3 (注意: --get-raw-02 不使用这些参数)").arg(led).arg(gain).arg(exposure));
	DfSetParamLedCurrent(led);
	DfSetParamCameraExposure(exposure);
	DfSetParamCameraGain(gain);

	QStringList param_confirm;
	logCurrentParamsInto(u8"标定拍照前(断开前确认)", param_confirm);
	logConsoleOnly(param_confirm.join('\n'));

	// --get-raw-02 opens its OWN connection to the camera -- the camera very likely only
	// accepts one client at a time, so our in-process DfConnect session has to step aside first.
	lines << u8"断开当前连接，交由外部程序采集...";
	DfDisconnect(ip.toStdString().c_str());
	connected_ = false;

	QDir().mkpath(save_folder);

	QStringList args;
	args << "--get-raw-02" << "--ip" << ip << "--path" << QDir::toNativeSeparators(save_folder);

	logConsoleOnly(QString(u8"运行: open_cam3d.exe %1").arg(args.join(' ')));

	QString exe_output;
	int exit_code = runExeBlocking("open_cam3d.exe", args, exe_output, 60000);
	logConsoleOnly(exe_output); // raw exe stdout -- verbose connection trace, console only
	lines << QString(u8"采集完成 (退出码 %1)").arg(exit_code);

	lines << u8"重新连接...";
	int reconnect_ret = DfConnect(ip.toStdString().c_str());

	QImage out_image;

	if (reconnect_ret == DF_SUCCESS)
	{
		connected_ = true;
		DfSetCaptureEngine(XemaEngine::Black);
		DfSetParamLedCurrent(led);
		DfSetParamCameraExposure(exposure);
		DfSetParamCameraGain(gain);
		lines << u8"重新连接成功，已恢复黑白引擎与当前参数";

		QString preview_path = save_folder + "/phase36.bmp";
		cv::Mat gray = cv::imread(preview_path.toStdString(), cv::IMREAD_GRAYSCALE);

		if (!gray.empty())
		{
			lines << QString(u8"标定图像已保存: %1").arg(save_folder);
			out_image = grayToQImage(gray);

			if (exit_code == 0)
			{
				calib_pose_index_++;
			}
		}
		else
		{
			lines << QString(u8"警告: 未能读取预览图像 %1，请检查采集是否成功").arg(preview_path);
		}
	}
	else
	{
		lines << QString(u8"重新连接失败，错误码: %1 -- 请手动点击连接").arg(reconnect_ret);
	}

	// Auto-resumes continuous capture afterward if it was running before.
	if (was_capturing && connected_)
	{
		capturing_ = true;
		std::thread resume_t(&XemaCameraWindow::captureLoopThreadFunc, this);
		resume_t.detach();
		lines << u8"连续采集已自动恢复";
	}

	emit calibCaptureFinished(lines.join('\n'), out_image);
}

void XemaCameraWindow::onCalibCaptureFinished(QString message, QImage image)
{
	log(message);

	if (!image.isNull())
	{
		label_image_->setPixmap(QPixmap::fromImage(image).scaled(
			label_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
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
		log(u8"标定正在进行中，请稍候...");
		return;
	}

	QString identity_folder = currentIdentityFolder();

	if (!QDir(identity_folder).exists())
	{
		log(QString(u8"未找到 %1 -- 请先用「拍照（用于标定）」采集至少几组姿态").arg(identity_folder));
		return;
	}

	calibrating_ = true;
	btn_calibrate_->setEnabled(false);
	startBusyHeartbeat(label_calib_status_, u8"标定中");

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

	logConsoleOnly(QString(u8"运行: calibration.exe %1").arg(args.join(' ')));

	QString exe_output;
	// Calibration across many poses can genuinely take minutes -- much longer timeout than
	// the single-frame raw capture uses.
	int exit_code = runExeBlocking("calibration.exe", args, exe_output, 600000);
	logConsoleOnly(exe_output); // full board-detection/reprojection detail -- console only

	bool ok = QFile::exists(calib_path);
	QString message = ok
		? QString(u8"标定完成: %1  (退出码 %2，仅供参考)").arg(calib_path).arg(exit_code)
		: QString(u8"标定失败 -- 未生成 %1 (退出码 %2)").arg(calib_path).arg(exit_code);

	emit calibrateFinished(message, ok);
}

void XemaCameraWindow::onCalibrateFinished(QString message, bool ok)
{
	stopBusyHeartbeat();
	log(message);

	calibrating_ = false;
	btn_calibrate_->setEnabled(true);

	if (ok)
	{
		label_calib_status_->setText(u8"标定: 完成");
		label_calib_status_->setStyleSheet("color: green; font-weight: bold;");
	}
	else
	{
		label_calib_status_->setText(u8"标定: 失败");
		label_calib_status_->setStyleSheet("color: red; font-weight: bold;");
	}
}

// ==================== write params ====================

void XemaCameraWindow::onWriteParamsClicked()
{
	if (!connected_)
	{
		log(u8"请先连接相机");
		return;
	}

	if (writing_params_)
	{
		log(u8"写参数正在进行中，请稍候...");
		return;
	}

	QString ip = edit_ip_->text().trimmed();
	if (ip.isEmpty())
	{
		log(u8"请输入相机IP");
		return;
	}

	QString calib_path = currentIdentityFolder() + "/param.txt";
	if (!QFile::exists(calib_path))
	{
		log(QString(u8"未找到 %1 -- 请先成功运行标定").arg(calib_path));
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
	startBusyHeartbeat(label_write_status_, u8"写入中");

	std::thread t(&XemaCameraWindow::writeParamsThreadFunc, this, ip, calib_path, led, gain, exposure);
	t.detach();
}

void XemaCameraWindow::writeParamsThreadFunc(QString ip, QString calib_path, int led, float gain, float exposure)
{
	QStringList lines;
	bool was_capturing = capturing_;

	if (was_capturing)
	{
		lines << u8"正在停止连续采集以写入参数...";
		capturing_ = false;

		QElapsedTimer wait_timer;
		wait_timer.start();
		while (capture_thread_active_ && wait_timer.elapsed() < kCaptureStopTimeoutMs)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if (capture_thread_active_)
		{
			lines << QString(u8"警告: 等待采集停止超时 (%1ms)，仍继续写入参数 -- 可能短暂卡顿").arg(kCaptureStopTimeoutMs);
		}
	}

	DfSetParamLedCurrent(led);
	DfSetParamCameraExposure(exposure);
	DfSetParamCameraGain(gain);

	lines << u8"断开当前连接，交由外部程序写入参数...";
	DfDisconnect(ip.toStdString().c_str());
	connected_ = false;

	QStringList args;
	args << "--set-calib-looktable" << "--ip" << ip << "--path" << QDir::toNativeSeparators(calib_path);

	logConsoleOnly(QString(u8"运行: open_cam3d.exe %1").arg(args.join(' ')));

	QString exe_output;
	int exit_code = runExeBlocking("open_cam3d.exe", args, exe_output, 60000);
	logConsoleOnly(exe_output);
	lines << QString(u8"写参数完成 (退出码 %1)").arg(exit_code);

	lines << u8"重新连接...";
	int reconnect_ret = DfConnect(ip.toStdString().c_str());

	bool ok = false;

	if (reconnect_ret == DF_SUCCESS)
	{
		connected_ = true;
		DfSetCaptureEngine(XemaEngine::Black);
		DfSetParamLedCurrent(led);
		DfSetParamCameraExposure(exposure);
		DfSetParamCameraGain(gain);
		lines << u8"重新连接成功，已恢复黑白引擎与当前参数";

		ok = (exit_code == 0);

		if (was_capturing)
		{
			capturing_ = true;
			std::thread resume_t(&XemaCameraWindow::captureLoopThreadFunc, this);
			resume_t.detach();
			lines << u8"连续采集已自动恢复";
		}
	}
	else
	{
		lines << QString(u8"重新连接失败，错误码: %1 -- 请手动点击连接").arg(reconnect_ret);
	}

	emit writeParamsFinished(lines.join('\n'), ok);
}

void XemaCameraWindow::onWriteParamsFinished(QString message, bool ok)
{
	stopBusyHeartbeat();
	log(message);

	writing_params_ = false;
	setConnectedUiState(connected_);
	if (connected_)
	{
		label_firmware_->setText(u8"固件版本: 已连接");
	}
	btn_capture_->setText(capturing_ ? u8"停止采集" : u8"开始连续采集");

	if (ok)
	{
		label_write_status_->setText(u8"写参数: 完成");
		label_write_status_->setStyleSheet("color: green; font-weight: bold;");
	}
	else
	{
		label_write_status_->setText(u8"写参数: 失败");
		label_write_status_->setStyleSheet("color: red; font-weight: bold;");
	}
}