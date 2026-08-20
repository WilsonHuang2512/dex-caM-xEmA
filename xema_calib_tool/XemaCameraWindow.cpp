#include "XemaCameraWindow.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QApplication>
#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <thread>
#include <cmath>
#include <QElapsedTimer>
#include <QAbstractButton>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

XemaCameraWindow::XemaCameraWindow(QWidget* parent)
	: QWidget(parent)
{
	setWindowTitle(u8"XEMA 相机控制");
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

	// ---- params ----
	spin_led_ = new QSpinBox(this);
	spin_led_->setRange(0, 1023);
	spin_led_->setValue(1023);

	spin_gain_ = new QDoubleSpinBox(this);
	spin_gain_->setRange(0.0, 24.0); // matches the real GUI's doubleSpinBox_gain range
	spin_gain_->setSingleStep(0.1);
	spin_gain_->setValue(0.0);

	spin_exposure_ = new QDoubleSpinBox(this);
	// Real range depends on which projector is attached (3010: 1700-100000, 4710:
	// 1700-28000, confirmed from camera_capture_gui.cpp's setCameraConfigParam()) -- this is
	// just a placeholder until DfGetProjectorVersion comes back after connecting;
	// applyExposureRangeForProjector() narrows it to the real bounds then.
	spin_exposure_->setRange(1700.0, 100000.0);
	spin_exposure_->setDecimals(0);
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

	// ---- capture engine ----
	// XemaEngine::Normal/Reflect/Black -- DfCaptureData uses a genuinely different
	// reconstruction path per engine (see header comment). Defaulting the UI to Normal here
	// rather than the SDK's raw Reflect default is a starting guess for testing against
	// whatever the real vendor GUI is actually using -- switch this and re-test board
	// recognition if Normal doesn't match.
	combo_engine_ = new QComboBox(this);
	combo_engine_->addItem(u8"普通 (Normal)");
	combo_engine_->addItem(u8"反光 (Reflect)");
	combo_engine_->addItem(u8"黑白 (Black)");
	combo_engine_->setCurrentIndex(0);

	QHBoxLayout* engine_row = new QHBoxLayout();
	engine_row->addWidget(new QLabel(u8"采集引擎:", this));
	engine_row->addWidget(combo_engine_);
	engine_row->addStretch();

	// ---- board spacing ----
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
	btn_save_frame_ = new QPushButton(u8"保存当前帧", this);
	btn_save_frame_->setEnabled(false);
	btn_save_frame_->setToolTip(u8"保存最近一次采集的原始亮度图（未叠加过曝/标定板标记），用于检查图像本身");
	label_overexposure_ = new QLabel(u8"过曝: -", this);
	label_board_ = new QLabel(u8"标定板: -", this);

	QHBoxLayout* capture_row = new QHBoxLayout();
	capture_row->addWidget(btn_capture_);
	capture_row->addWidget(btn_save_frame_);
	capture_row->addWidget(label_overexposure_);
	capture_row->addWidget(label_board_);

	label_image_ = new QLabel(this);
	label_image_->setMinimumHeight(350);
	label_image_->setAlignment(Qt::AlignCenter);
	label_image_->setStyleSheet("background-color:#222; border: 1px solid #888;");
	label_image_->setText(u8"（未连接）");
	label_image_->setStyleSheet("background-color:#222; color:#aaa; border: 1px solid #888;");

	// ---- log ----
	log_view_ = new QTextEdit(this);
	log_view_->setReadOnly(true);
	log_view_->setFixedHeight(140);

	QVBoxLayout* main_layout = new QVBoxLayout(this);
	main_layout->addLayout(connect_row);
	main_layout->addWidget(param_box);
	main_layout->addLayout(engine_row);
	main_layout->addWidget(board_box);
	main_layout->addLayout(capture_row);
	main_layout->addWidget(label_image_, 1);
	main_layout->addWidget(log_view_);

	connect(btn_connect_, &QPushButton::clicked, this, &XemaCameraWindow::onConnectClicked);
	connect(btn_disconnect_, &QPushButton::clicked, this, &XemaCameraWindow::onDisconnectClicked);
	connect(btn_apply_params_, &QPushButton::clicked, this, &XemaCameraWindow::onApplyParamsClicked);
	connect(btn_capture_, &QPushButton::clicked, this, &XemaCameraWindow::onCaptureToggled);
	connect(btn_save_frame_, &QPushButton::clicked, this, &XemaCameraWindow::onSaveFrameClicked);
	connect(group_board_spacing_, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked), this, &XemaCameraWindow::onBoardSpacingChanged);
	connect(combo_engine_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &XemaCameraWindow::onCaptureEngineChanged);
	connect(this, &XemaCameraWindow::connectFinished, this, &XemaCameraWindow::onConnectFinished);
	connect(this, &XemaCameraWindow::captureFinished, this, &XemaCameraWindow::onCaptureFinished);
	connect(this, &XemaCameraWindow::applyParamsFinished, this, &XemaCameraWindow::onApplyParamsFinished);
	connect(this, &XemaCameraWindow::disconnectFinished, this, &XemaCameraWindow::onDisconnectFinished);

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
}

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

	int spacing = obj.value("board_spacing_mm").toInt(80);
	switch (spacing)
	{
	case 4: radio_spacing_4_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing4; break;
	case 12: radio_spacing_12_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing12; break;
	case 20: radio_spacing_20_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing20; break;
	case 40: radio_spacing_40_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing40; break;
	default: radio_spacing_80_->setChecked(true); board_spacing_ = XemaBoardSpacingMm::Spacing80; break;
	}

	int engine = obj.value("capture_engine").toInt((int)XemaEngine::Normal);
	if (engine >= 0 && engine <= 2)
	{
		capture_engine_ = engine;
		combo_engine_->setCurrentIndex(engine);
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
	obj["capture_engine"] = capture_engine_;

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

void XemaCameraWindow::onCaptureEngineChanged(int index)
{
	if (index < 0 || index > 2) return;

	capture_engine_ = index;
	saveConfig();

	if (!connected_) return;

	// DfSetCaptureEngine is a plain local variable set inside the SDK (no network call, no
	// mutex) -- safe to call directly here even mid-capture, unlike DfSetParamCameraExposure/
	// DfSetParamCameraGain which needed the stop/resume dance in applyParamsThreadFunc.
	XemaEngine engine = (XemaEngine)index;
	int ret = DfSetCaptureEngine(engine);
	const char* names[3] = { u8"普通(Normal)", u8"反光(Reflect)", u8"黑白(Black)" };
	log(QString(u8"采集引擎切换为: %1  (返回码 %2)").arg(names[index]).arg(ret));
}

void XemaCameraWindow::log(const QString& msg)
{
	log_view_->append(msg);
}

void XemaCameraWindow::setConnectedUiState(bool connected)
{
	connected_ = connected;
	edit_ip_->setEnabled(!connected);
	btn_connect_->setEnabled(!connected);
	btn_disconnect_->setEnabled(connected);
	btn_apply_params_->setEnabled(connected);
	btn_capture_->setEnabled(connected);
	btn_save_frame_->setEnabled(connected);
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
	// raw Bayer mosaic that needs de-mosaicing (BayerRG8 sensor) -- see header comment.
	// Confirmed from camera_capture_gui.cpp's connect handler, which does exactly this and
	// defaults to Mono if the query fails (same fallback used here).
	int pixel_type_ret = DfGetCameraPixelType(pixel_type_);
	if (pixel_type_ret != DF_SUCCESS)
	{
		pixel_type_ = (int)XemaPixelType::Mono;
	}

	char fw_buf[64] = { 0 };
	int fw_ret = DfGetFirmwareVersion(fw_buf);
	QString fw = (fw_ret == DF_SUCCESS) ? QString::fromUtf8(fw_buf) : QString(u8"获取失败");

	QString sensor_desc = (pixel_type_ == (int)XemaPixelType::BayerRG8) ? u8"彩色(Bayer)" : u8"黑白(Mono)";

	// Explicitly apply the configured capture engine right after connecting, rather than
	// relying on the combo box's change signal having fired at the right time (it may have
	// fired earlier, while connected_ was still false and thus skipped the DfSetCaptureEngine
	// call). This guarantees the engine is actually set to what the UI shows, every connect.
	DfSetCaptureEngine((XemaEngine)capture_engine_);
	const char* engine_names[3] = { u8"普通(Normal)", u8"反光(Reflect)", u8"黑白(Black)" };
	QString engine_desc = (capture_engine_ >= 0 && capture_engine_ <= 2) ? engine_names[capture_engine_] : u8"未知";

	emit connectFinished(true, QString(u8"连接成功。分辨率: %1x%2  固件: %3  光机型号: %4  传感器: %5  采集引擎: %6")
		.arg(width_).arg(height_).arg(fw).arg(projector_version_).arg(sensor_desc).arg(engine_desc));
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
	// DfGetParamCameraGain -- these are the params that actually affect DfCaptureData (the
	// full structured-light capture this GUI uses), NOT DfGetParamGenerateBrightness/
	// DfGetParamBrightnessGain (those are for the separate DfCaptureBrightnessData path,
	// which this GUI doesn't call -- see header comment for the earlier bug this caused).
	// Fresh queries, not just echoing back what we last sent, so if a Set call silently
	// didn't take effect this readback shows the camera's real current value.
	// Appends to lines_out instead of calling log() directly -- this function is called from
	// background threads (applyParamsThreadFunc), and QTextEdit isn't thread-safe.
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
		// pull the firmware line back out of the combined message for the label -- simplest
		// to just show a generic "connected" state here since the full detail is in the log
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
	label_overexposure_->setText(u8"过曝: -");
	label_board_->setText(u8"标定板: -");
	label_board_->setStyleSheet("");

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

	// DfSetParamCameraExposure/DfSetParamCameraGain -- these are what actually affects
	// DfCaptureData's output (confirmed from the real GUI's do_spin_camera_exposure_changed /
	// do_doubleSpin_gain handlers). Only safe to call now that the capture loop has actually
	// stopped (capture_thread_active_ == false) -- DfCaptureData holds the SDK's internal
	// command mutex for the whole capture, so calling these while a capture is still
	// in-flight is what caused the GUI-freeze bug this function exists to avoid.
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

	// Read straight back from the camera rather than trusting the Set calls' return codes
	// alone -- a SUCCESS return code doesn't guarantee the firmware actually applied it.
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

QImage XemaCameraWindow::matToQImage(const cv::Mat& mat_bgr)
{
	cv::Mat rgb;
	cv::cvtColor(mat_bgr, rgb, cv::COLOR_BGR2RGB);
	return QImage(rgb.data, rgb.cols, rgb.rows, (int)rgb.step, QImage::Format_RGB888).copy();
}

double XemaCameraWindow::markOverexposure(const cv::Mat& gray, cv::Mat& overlay_bgr_out)
{
	cv::cvtColor(gray, overlay_bgr_out, cv::COLOR_GRAY2BGR);

	cv::Mat mask;
	cv::threshold(gray, mask, kOverexposureThreshold, 255, cv::THRESH_BINARY);
	overlay_bgr_out.setTo(cv::Scalar(0, 0, 255), mask); // BGR red

	int overexposed_count = cv::countNonZero(mask);
	double percent = 100.0 * overexposed_count / ((double)gray.rows * gray.cols);
	return percent;
}

cv::Size XemaCameraWindow::boardGridSize() const
{
	// (cols, rows) -- see the ASSUMPTION comment on this declaration in the header
	return cv::Size(9, 13);
}

bool XemaCameraWindow::detectBoard(const cv::Mat& gray, std::vector<cv::Point2f>& points_out)
{
	if (gray.empty())
	{
		return false;
	}

	cv::Mat inv;
	cv::bitwise_not(gray, inv);

	cv::Size board_size = boardGridSize();
	const int flags = cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;

	cv::SimpleBlobDetector::Params p;
	p.minArea = 40;
	p.maxArea = 5000;
	cv::Ptr<cv::SimpleBlobDetector> det = cv::SimpleBlobDetector::create(p);
	if (cv::findCirclesGrid(inv, board_size, points_out, flags, det))
	{
		return true;
	}

	cv::Ptr<cv::SimpleBlobDetector> default_det = cv::SimpleBlobDetector::create();
	if (cv::findCirclesGrid(inv, board_size, points_out, flags, default_det))
	{
		return true;
	}

	return cv::findCirclesGrid(inv, board_size, points_out, cv::CALIB_CB_ASYMMETRIC_GRID, det);
}

void XemaCameraWindow::onSaveFrameClicked()
{
	cv::Mat frame_copy;
	{
		std::lock_guard<std::mutex> lock(last_raw_gray_mutex_);
		if (last_raw_gray_.empty())
		{
			log(u8"还没有采集过任何帧，无法保存");
			return;
		}
		frame_copy = last_raw_gray_.clone();
	}

	QString filename = QCoreApplication::applicationDirPath() + "/"
		+ QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + "_raw_frame.bmp";

	bool ok = cv::imwrite(filename.toStdString(), frame_copy);
	if (ok)
	{
		log(QString(u8"原始帧已保存: %1").arg(filename));
	}
	else
	{
		log(QString(u8"保存失败: %1").arg(filename));
	}
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
		log(u8"========== 开始连续采集 ==========");
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
		// exposure_num=1 matches the real GUI's non-HDR continuous-capture path
		// (captureOneFrameBaseThread(false)). Previously this called
		// DfCaptureBrightnessData directly, a lightweight standalone grab that skips pattern
		// projection entirely -- see header comment for why that looked different/faster.
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
		// DfGetBrightnessData ALREADY does pixel-type-aware Bayer->RGB->Gray conversion
		// internally -- confirmed directly from open_cam3d.cpp's own implementation (the
		// exact one this build links, not the separate cpp/xema_camera.cpp C++ wrapper):
		// pixel_type_==Mono memcpy's directly, pixel_type_==BayerRG8 calls
		// DfBayerToRgb+DfRgbToGray before returning. There is no raw-Bayer-passthrough case.
		// An earlier version of this code second-guessed that and routed BayerRG8 through
		// DfGetColorBrightnessData(Rgb) + our own cv::cvtColor instead -- unnecessary, and
		// actively wrong for XemaEngine::Black specifically: Black mode's brightness_buf_
		// is a mono-bypass capture, not real Bayer-patterned data, so de-mosaicing it with
        // DfBayerToRgb (keyed only on the sensor's static pixel_type_, not the active
        // engine) would have scrambled it. Just call this plain and let the SDK handle it.
		int brightness_ret = DfGetBrightnessData(gray.data);

		if (brightness_ret != DF_SUCCESS)
		{
			emit captureFinished(false, QString(u8"获取亮度图失败，错误码: %1").arg(brightness_ret), QImage());
			continue;
		}

		{
			std::lock_guard<std::mutex> lock(last_raw_gray_mutex_);
			last_raw_gray_ = gray.clone(); // exact SDK output, nothing drawn on it -- for "保存当前帧"
		}

		cv::Mat overlay;
		double overexposed_percent = markOverexposure(gray, overlay);

		std::vector<cv::Point2f> board_points;
		bool board_found = detectBoard(gray, board_points);
		if (board_found)
		{
			// Only draw when actually found -- drawChessboardCorners's not-found path draws
			// every candidate point as a red X, which visually merges into the red
			// overexposure overlay and makes the whole frame look uniformly red/confusing.
			// Skipping the draw entirely when not found avoids that clash outright.
			cv::drawChessboardCorners(overlay, boardGridSize(), board_points, true);
		}

		QString message = u8"采集完成。";
		if (overexposed_percent >= kOverexposureWarnPercent)
		{
			message = QString(u8"采集完成 -- 警告: 过曝 %1%% 的像素").arg(overexposed_percent, 0, 'f', 2);
		}
		// board status is tucked into the message with a recognizable prefix rather than a
		// new signal parameter -- onCaptureFinished parses it back out to update the board
		// label. Keeps the existing captureFinished(bool,QString,QImage) signature stable.
		message += board_found ? u8" [BOARD:FOUND]" : u8" [BOARD:NONE]";

		emit captureFinished(true, message, matToQImage(overlay));
	}

	capture_thread_active_ = false;
}

void XemaCameraWindow::onCaptureFinished(bool ok, QString message, QImage image)
{
	// Board status is tucked into the message as a recognizable suffix (see
	// captureLoopThreadFunc) -- pull it out here and strip it before logging/matching, so
	// the log and the overexposure-text check above don't see the marker as extra content.
	bool board_found = false;
	bool has_board_marker = false;
	if (message.contains(u8"[BOARD:FOUND]"))
	{
		board_found = true;
		has_board_marker = true;
		message.remove(u8" [BOARD:FOUND]");
	}
	else if (message.contains(u8"[BOARD:NONE]"))
	{
		has_board_marker = true;
		message.remove(u8" [BOARD:NONE]");
	}

	// During continuous capture, logging every single successful frame would flood the log
	// view -- only failures and overexposure warnings get a log line. The image and
	// overexposure/board labels still update every frame regardless, so nothing silent is lost.
	if (!ok || message.contains(u8"过曝") || message.contains(u8"失败"))
	{
		log(message);
	}

	if (ok && !image.isNull())
	{
		label_image_->setPixmap(QPixmap::fromImage(image).scaled(
			label_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

		if (message.contains(u8"过曝"))
		{
			label_overexposure_->setText(u8"过曝: 警告");
			label_overexposure_->setStyleSheet("color: red; font-weight: bold;");
		}
		else
		{
			label_overexposure_->setText(u8"过曝: 正常");
			label_overexposure_->setStyleSheet("");
		}

		if (has_board_marker)
		{
			if (board_found)
			{
				label_board_->setText(u8"标定板: 识别到");
				label_board_->setStyleSheet("color: green; font-weight: bold;");
			}
			else
			{
				label_board_->setText(u8"标定板: 未识别");
				label_board_->setStyleSheet("");
			}
		}
	}
}