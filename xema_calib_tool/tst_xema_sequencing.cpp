// tst_xema_sequencing.cpp
//
// Tier 2B (see xema_calib_tool_test_plan.md): full reconnect/resume sequencing, deterministic
// and instant because it links fake_open_cam3d_sdk instead of the real SDK -- no real sockets,
// no TEST-NET timeouts. This is the piece Tier 2A (tst_xema_click_races.cpp) structurally
// couldn't cover: with the real SDK, DfConnect to an unreachable TEST-NET address always
// fails, so Tier 2A could only prove the GUARD FLAGS work, never what a SUCCESSFUL
// disconnect/reconnect sequence actually does. Here, DfConnect/DfDisconnect/etc. succeed
// instantly and log every call, so we can assert on the exact sequence.
//
// Build wiring: links fake_open_cam3d_sdk (NOT open_cam3d_sdk) -- see CMakeLists.txt's
// tst_xema_sequencing target. XemaCameraWindow.cpp/.h are completely unmodified; they include
// the same real open_cam3d.h either way, just resolved against a different DLL at link time.
//
// Safety note: unlike tst_xema_click_races.cpp, there's no TEST-NET-address requirement here
// -- DfConnect never touches a real network at all with the fake SDK linked in, so any string
// is safe to use as the "IP".

#include <QtTest/QtTest>
#include <QApplication>
#include <QTemporaryDir>
#include <QFile>
#include <QDir>
#include "XemaCameraWindow.h"
#include "fake_sdk_control.h"

namespace
{
	QStringList GetCallLog()
	{
		QStringList log;
		int count = FakeSdk_GetCallCount();
		char buf[256];
		for (int i = 0; i < count; ++i)
		{
			FakeSdk_GetCallAt(i, buf, sizeof(buf));
			log << QString::fromUtf8(buf);
		}
		return log;
	}

	// Index of the first call log entry whose function name starts with `prefix`, or -1.
	int FindCall(const QStringList& log, const QString& prefix)
	{
		for (int i = 0; i < log.size(); ++i)
		{
			if (log[i].startsWith(prefix)) return i;
		}
		return -1;
	}

	QString MakeFakeIdentityFolderWithParam(const QDir& root, const QString& identity)
	{
		QString folder = root.filePath(identity);
		QDir().mkpath(folder + "/00");
		QFile param_file(folder + "/param.txt");
		param_file.open(QIODevice::WriteOnly);
		param_file.write("fake param.txt for sequencing test\n");
		return folder;
	}
}

class TestXemaSequencing : public QObject
{
	Q_OBJECT

private slots:
	void init()
	{
		FakeSdk_Reset();
		temp_dir_ = new QTemporaryDir();
		QVERIFY(temp_dir_->isValid());
	}

	void cleanup()
	{
		delete temp_dir_;
		temp_dir_ = nullptr;
	}

	// ---------------- Correct: full sequencing ----------------

	void correctSequencing_whenConnected_disconnectsThenReconnectsAndRestoresParams()
	{
		XemaCameraWindow w;
		QDir root(temp_dir_->path());
		w.testHook_setSavePath(root.path());
		w.testHook_setIdentity("cam1");
		w.testHook_setIp("fake-camera-1"); // never touches a real network with the fake SDK linked
		MakeFakeIdentityFolderWithParam(root, "cam1");

		w.testHook_forceConnectedStateForTest(true);

		QMetaObject::invokeMethod(&w, "onCorrectClicked");
		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isCorrecting(), 5000);

		QStringList log = GetCallLog();
		int disconnect_idx = FindCall(log, "DfDisconnect");
		int connect_idx = FindCall(log, "DfConnect(");
		int engine_idx = FindCall(log, "DfSetCaptureEngine");
		int led_idx = FindCall(log, "DfSetParamLedCurrent");
		int exposure_idx = FindCall(log, "DfSetParamCameraExposure");
		int gain_idx = FindCall(log, "DfSetParamCameraGain");

		QVERIFY2(disconnect_idx >= 0, "expected DfDisconnect when starting from a connected state");
		QVERIFY2(connect_idx >= 0, "expected a reconnect DfConnect call");
		QVERIFY2(disconnect_idx < connect_idx, "disconnect must happen BEFORE reconnect, not after");

		// The "restore Black engine + current LED/exposure/gain" step must happen AFTER the
		// reconnect succeeds, not before (there'd be nothing connected to apply them to yet).
		QVERIFY2(engine_idx > connect_idx, "capture engine restore must follow the reconnect");
		QVERIFY2(led_idx > connect_idx, "LED restore must follow the reconnect");
		QVERIFY2(exposure_idx > connect_idx, "exposure restore must follow the reconnect");
		QVERIFY2(gain_idx > connect_idx, "gain restore must follow the reconnect");
	}

	void correctSequencing_whenNotConnected_neverTouchesConnectionAtAll()
	{
		// Regression test for the exact behavior fixed earlier in this project: Correct run
		// without ever having connected this session must NOT disconnect (nothing to step
		// aside for) and must NOT auto-reconnect afterward either (shouldn't silently connect
		// the camera behind the user's back).
		XemaCameraWindow w;
		QDir root(temp_dir_->path());
		w.testHook_setSavePath(root.path());
		w.testHook_setIdentity("cam1");
		w.testHook_setIp("fake-camera-1");
		MakeFakeIdentityFolderWithParam(root, "cam1");

		// Deliberately NOT calling testHook_forceConnectedStateForTest -- connected_ starts false.

		QMetaObject::invokeMethod(&w, "onCorrectClicked");
		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isCorrecting(), 5000);

		QStringList log = GetCallLog();
		QVERIFY2(FindCall(log, "DfDisconnect") < 0, "must not disconnect when nothing was connected");
		QVERIFY2(FindCall(log, "DfConnect(") < 0, "must not silently auto-connect when the session started disconnected");
		QVERIFY2(FindCall(log, "DfSetCaptureEngine") < 0, "must not touch capture engine without a connection");
	}

	// ---------------- Calibrate: confirms it's genuinely SDK-free ----------------

	void calibrateSequencing_neverCallsAnySdkFunction()
	{
		// calibrateThreadFunc's own comment claims this is "pure file-based computation: no
		// camera connection involved at all" -- this test holds that claim to account. If a
		// future change accidentally introduces an SDK call into the calibrate path, this
		// fails immediately instead of only showing up as a subtle behavior change.
		XemaCameraWindow w;
		QDir root(temp_dir_->path());
		w.testHook_setSavePath(root.path());
		w.testHook_setIdentity("cam1");
		QDir().mkpath(root.filePath("cam1/00")); // pose folder only, no param.txt needed for calibrate

		QMetaObject::invokeMethod(&w, "onCalibrateClicked");
		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isCalibrating(), 5000);

		QCOMPARE(FakeSdk_GetCallCount(), 0);
	}

private:
	QTemporaryDir* temp_dir_ = nullptr;
};

QTEST_MAIN(TestXemaSequencing)
#include "tst_xema_sequencing.moc"
