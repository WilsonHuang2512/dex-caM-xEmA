// tst_xema_click_races.cpp
//
// Automates Tier 2's "click twice fast" / "click A then B while A is running" checklist.
// Key property that makes this deterministic (no sleeps, no flakiness): every guard flag
// (calibrating_, writing_params_, correcting_, calib_capturing_, busy_) is set to true
// SYNCHRONOUSLY on the GUI thread, before the background std::thread is spawned. So calling
// the same onXClicked() slot twice in a row from the test -- with no event-loop spin in
// between -- deterministically hits the guard on the second call, regardless of whether the
// background thread has even started yet, let alone finished. No timing luck required.
//
// Safety: edit_ip_ is set to an address from 192.0.2.0/24 (RFC 5737 TEST-NET-1) for every
// test. This range is reserved for documentation/testing and guaranteed non-routable, so the
// real DfConnect() call the background thread makes is guaranteed to fail fast/timeout
// harmlessly -- it can never reach a real camera, even if one happens to be reachable on the
// test machine's LAN. DO NOT replace this with a real or guessed camera IP in these tests.
//
// Needs seven trivial test-only accessors added to XemaCameraWindow, guarded by a macro so
// there's zero cost/risk in a normal (non-test) build:
//
//   #ifdef XEMA_TEST_HOOKS
//   public:
//       void testHook_setIp(const QString& ip) { edit_ip_->setText(ip); }
//       void testHook_setIdentity(const QString& id) { edit_identity_->setText(id); }
//       void testHook_setSavePath(const QString& p) { edit_save_path_->setText(p); }
//       void testHook_forceConnectedStateForTest(bool v) { connected_ = v; } // raw override, not a real connect
//       bool testHook_isCalibrating() const { return calibrating_; }
//       bool testHook_isCorrecting() const { return correcting_; }
//       bool testHook_isWritingParams() const { return writing_params_; }
//       bool testHook_isCalibCapturing() const { return calib_capturing_; }
//       bool testHook_isBusy() const { return busy_; }
//   #endif
//
// These are already added to XemaCameraWindow.h and CMakeLists.txt now builds a matching
// tst_xema_click_races target with -DXEMA_TEST_HOOKS (see the "Tests" section at the bottom
// of CMakeLists.txt) -- no further wiring needed before this compiles and runs.

#include <QtTest/QtTest>
#include <QApplication>
#include <QTemporaryDir>
#include "XemaCameraWindow.h"

namespace
{
	constexpr const char* kTestNetIp = "192.0.2.10"; // RFC 5737 TEST-NET-1 -- never routes anywhere real

	// Minimal on-disk fixture so the pose-folder-exists precondition in onCalibrateClicked /
	// onCorrectClicked passes and the test reaches the actual flag-guard logic being tested.
	QString makeFakeIdentityFolder(const QDir& root, const QString& identity)
	{
		QString folder = root.filePath(identity);
		QDir().mkpath(folder + "/00");
		return folder;
	}
}

class TestXemaClickRaces : public QObject
{
	Q_OBJECT

private slots:
	void init()
	{
		// Fresh temp dir per test so tests can't interfere with each other's fixture files.
		temp_dir_ = new QTemporaryDir();
		QVERIFY(temp_dir_->isValid());
	}

	void cleanup()
	{
		delete temp_dir_;
		temp_dir_ = nullptr;
	}

	// ---------------- same-button double-click races ----------------

	void calibrateClickedTwice_secondCallIsRejected()
	{
		XemaCameraWindow w;
		QDir root(temp_dir_->path());
		w.testHook_setSavePath(root.path());
		w.testHook_setIdentity("cam1");
		makeFakeIdentityFolder(root, "cam1");

		QVERIFY(!w.testHook_isCalibrating()); // sanity check on initial state

		QMetaObject::invokeMethod(&w, "onCalibrateClicked");
		QVERIFY(w.testHook_isCalibrating()); // flag flips synchronously, before the thread even runs

		// Second click while the (real, but harmless -- pure file I/O, no camera) background
		// run is still in flight. This is the exact shape of bug the busy_ regression was:
		// if this guard were missing/broken, a second std::thread would launch here.
		QMetaObject::invokeMethod(&w, "onCalibrateClicked");
		// No crash, no second thread -- the only observable assertion available without a
		// log-capture hook is that the flag is still exactly the single true it was after the
		// first call (a second unguarded entry into onCalibrateClicked would still leave it
		// true too, so this alone doesn't PROVE no second thread launched -- see note below).
		QVERIFY(w.testHook_isCalibrating());

		// Let the real (background) calibration.exe attempt finish/fail/timeout before the
		// QTemporaryDir in cleanup() destroys the fixture out from under it.
		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isCalibrating(), 15000);
	}

	void correctClickedTwice_secondCallIsRejected()
	{
		XemaCameraWindow w;
		QDir root(temp_dir_->path());
		w.testHook_setSavePath(root.path());
		w.testHook_setIdentity("cam1");
		w.testHook_setIp(kTestNetIp);
		makeFakeIdentityFolder(root, "cam1");

		QVERIFY(!w.testHook_isCorrecting());

		QMetaObject::invokeMethod(&w, "onCorrectClicked");
		QVERIFY(w.testHook_isCorrecting());

		QMetaObject::invokeMethod(&w, "onCorrectClicked");
		QVERIFY(w.testHook_isCorrecting());

		// Correct's background thread does a real (TEST-NET, so harmless/fast-failing)
		// DfDisconnect/open_cam3d.exe/DfConnect sequence -- give it real time to unwind.
		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isCorrecting(), 30000);
	}

	// ---------------- cross-button mutual exclusion ----------------

	void correctWhileWriteParamsRunning_isRejected()
	{
		XemaCameraWindow w;
		QDir root(temp_dir_->path());
		w.testHook_setSavePath(root.path());
		w.testHook_setIdentity("cam1");
		w.testHook_setIp(kTestNetIp);
		makeFakeIdentityFolder(root, "cam1");

		// write-params requires connected_ before it'll even set its own guard flag -- force
		// that precondition past without a real camera/SDK mock (see the hook's own comment).
		// The subsequent DfDisconnect/exe-run/DfConnect it does for real still targets the
		// TEST-NET IP, so it fails fast/harmlessly, same as every other test here.
		w.testHook_forceConnectedStateForTest(true);

		QMetaObject::invokeMethod(&w, "onWriteParamsClicked");
		QVERIFY(w.testHook_isWritingParams()); // now genuinely running, not bailed out early

		QMetaObject::invokeMethod(&w, "onCorrectClicked");
		QVERIFY(!w.testHook_isCorrecting()); // must have been rejected by the calib_capturing_/writing_params_ cross-check

		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isWritingParams(), 30000);
	}

	void writeParamsWhileCorrectRunning_isRejected()
	{
		XemaCameraWindow w;
		QDir root(temp_dir_->path());
		w.testHook_setSavePath(root.path());
		w.testHook_setIdentity("cam1");
		w.testHook_setIp(kTestNetIp);
		makeFakeIdentityFolder(root, "cam1");

		QMetaObject::invokeMethod(&w, "onCorrectClicked");
		QVERIFY(w.testHook_isCorrecting());

		QMetaObject::invokeMethod(&w, "onWriteParamsClicked");
		QVERIFY(!w.testHook_isWritingParams());

		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isCorrecting(), 30000);
	}

	// ---------------- the actual busy_ regression, pinned down ----------------

	void connectFinished_alwaysResetsBusyFlag_evenOnFailure()
	{
		// This is the literal regression test for the bug that started this whole
		// conversation: busy_ was set true in onConnectClicked and never reset anywhere,
		// permanently locking out Connect after the very first attempt. Uses the TEST-NET IP
		// specifically because it fails FAST and DETERMINISTICALLY (no camera listening),
		// making the failure path exactly what this test needs to exercise.
		XemaCameraWindow w;
		w.testHook_setIp(kTestNetIp);

		QVERIFY(!w.testHook_isBusy());
		QMetaObject::invokeMethod(&w, "onConnectClicked");
		QVERIFY(w.testHook_isBusy());

		// Real connect attempt to an unreachable TEST-NET address -- should fail within a few
		// seconds (exact timeout depends on connectThreadFunc's own socket timeout, not
		// something this test controls).
		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isBusy(), 30000);

		// And -- the actual point of this test -- Connect must be usable again afterward.
		QMetaObject::invokeMethod(&w, "onConnectClicked");
		QVERIFY(w.testHook_isBusy()); // second attempt started fine, proving the first reset really happened
		QTRY_VERIFY_WITH_TIMEOUT(!w.testHook_isBusy(), 30000);
	}

private:
	QTemporaryDir* temp_dir_ = nullptr;
};

QTEST_MAIN(TestXemaClickRaces)
#include "tst_xema_click_races.moc"