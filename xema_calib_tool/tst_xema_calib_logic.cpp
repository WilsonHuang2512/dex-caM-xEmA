// tst_xema_calib_logic.cpp
//
// Tier 1 (see xema_calib_tool_test_plan.md): pure-logic tests, no camera, no exe, no
// background thread. Written against the real extraction done in XemaCameraWindow.cpp --
// buildCalibrateArgs/buildCorrectArgs/calibModeFromConfig/boardSpacingFromConfig are public
// static functions, callable directly with no instance needed. identityFolderName/
// currentIdentityFolder stay private (they read UI widgets) and go through the
// testHook_identityFolderName/testHook_currentIdentityFolder wrappers instead, same
// XEMA_TEST_HOOKS gating as tst_xema_click_races.cpp's hooks.
//
// Build: add to CMakeLists.txt the same way as tst_window_construct (needs
// XemaCameraWindow.cpp/.h, XEMA_TEST_HOOKS, Qt5::Widgets + Qt5::Test, OpenCV, open_cam3d_sdk,
// and xema_copy_runtime_dlls() -- see CMakeLists.txt's existing targets for the exact pattern):
//
//   add_executable(tst_xema_calib_logic tst_xema_calib_logic.cpp XemaCameraWindow.cpp XemaCameraWindow.h)
//   target_compile_definitions(tst_xema_calib_logic PRIVATE XEMA_TEST_HOOKS)
//   target_link_libraries(tst_xema_calib_logic PRIVATE Qt5::Widgets Qt5::Test ${OpenCV_LIBS} open_cam3d_sdk)
//   target_include_directories(tst_xema_calib_logic PRIVATE ${OpenCV_INCLUDE_DIRS} ${XEMA_SDK_DIR})
//   xema_copy_runtime_dlls(tst_xema_calib_logic)
//   if(MSVC)
//       target_compile_options(tst_xema_calib_logic PRIVATE /utf-8)
//   endif()
//
// Unlike tst_xema_click_races.cpp, these tests are fast (no real threads, no TEST-NET
// timeouts) -- the whole file should run in well under a second.

#include <QtTest/QtTest>
#include <QJsonObject>
#include "XemaCameraWindow.h"

class TestXemaCalibLogic : public QObject
{
	Q_OBJECT

private slots:
	// ---------------- identityFolderName ----------------

	void identityFolderName_empty_defaultsToDefault()
	{
		XemaCameraWindow w;
		w.testHook_setIdentity("");
		QCOMPARE(w.testHook_identityFolderName(), QString("default"));
	}

	void identityFolderName_macWithColons_replacedWithUnderscores()
	{
		XemaCameraWindow w;
		w.testHook_setIdentity("AA:BB:CC:DD:EE:FF");
		QCOMPARE(w.testHook_identityFolderName(), QString("AA_BB_CC_DD_EE_FF"));
	}

	void identityFolderName_macWithDashes_replacedWithUnderscores()
	{
		XemaCameraWindow w;
		w.testHook_setIdentity("AA-BB-CC-DD-EE-FF");
		QCOMPARE(w.testHook_identityFolderName(), QString("AA_BB_CC_DD_EE_FF"));
	}

	void identityFolderName_embeddedSpaces_replacedWithUnderscores()
	{
		XemaCameraWindow w;
		w.testHook_setIdentity("front cam 1");
		QCOMPARE(w.testHook_identityFolderName(), QString("front_cam_1"));
	}

	void identityFolderName_alreadyUnderscored_isIdempotent()
	{
		XemaCameraWindow w;
		w.testHook_setIdentity("front_cam_1");
		QCOMPARE(w.testHook_identityFolderName(), QString("front_cam_1"));
	}

	void identityFolderName_unicodeIdentity_isPassedThroughUnsanitized()
	{
		// Documents CURRENT behavior (no unicode handling at all) rather than asserting it's
		// correct -- flag for a product decision: is a raw Chinese-character folder name
		// actually fine on the target filesystems, or should this be transliterated/hashed?
		XemaCameraWindow w;
		w.testHook_setIdentity(QString::fromUtf8("前置相机"));
		QCOMPARE(w.testHook_identityFolderName(), QString::fromUtf8("前置相机"));
	}

	// ---------------- currentIdentityFolder ----------------

	void currentIdentityFolder_emptySavePath_fallsBackToExeDir()
	{
		XemaCameraWindow w;
		w.testHook_setSavePath("");
		w.testHook_setIdentity("cam1");
		QCOMPARE(w.testHook_currentIdentityFolder(),
			QCoreApplication::applicationDirPath() + "/cam1");
	}

	void currentIdentityFolder_savePathWithTrailingSlash_noDoubleSlash()
	{
		XemaCameraWindow w;
		w.testHook_setSavePath("C:/calib/XEMA/");
		w.testHook_setIdentity("cam1");
		// Documents current behavior -- currentIdentityFolder() does base_path + "/" + name
		// unconditionally, so a trailing slash in the save-path field produces "//". This
		// assertion is written to FAIL against that behavior (QVERIFY on the negation), so it
		// doubles as a flag: if this test starts failing, the double-slash bug got fixed
		// (intentionally or not) and this comment/assertion should be updated to reflect that.
		QVERIFY(w.testHook_currentIdentityFolder().contains("//"));
	}

	// ---------------- buildCalibrateArgs ----------------

	void buildCalibrateArgs_colorMode_usesPatternsC()
	{
		QStringList args = XemaCameraWindow::buildCalibrateArgs(
			"C:/calib/XEMA/cam1", 3010, 20, XemaCalibMode::Color, "C:/calib/XEMA/cam1/param.txt");
		int use_idx = args.indexOf("--use");
		QVERIFY(use_idx >= 0);
		QCOMPARE(args.value(use_idx + 1), QString("patterns-c"));
	}

	void buildCalibrateArgs_monoMode_usesPatterns()
	{
		QStringList args = XemaCameraWindow::buildCalibrateArgs(
			"C:/calib/XEMA/cam1", 3010, 20, XemaCalibMode::Mono, "C:/calib/XEMA/cam1/param.txt");
		int use_idx = args.indexOf("--use");
		QCOMPARE(args.value(use_idx + 1), QString("patterns"));
	}

	void buildCalibrateArgs_allBoardSpacings_mapCorrectly()
	{
		for (int spacing : { 4, 12, 20, 40, 80 })
		{
			QStringList args = XemaCameraWindow::buildCalibrateArgs(
				"C:/calib/XEMA/cam1", 3010, spacing, XemaCalibMode::Color, "C:/calib/XEMA/cam1/param.txt");
			int board_idx = args.indexOf("--board");
			QCOMPARE(args.value(board_idx + 1), QString::number(spacing));
		}
	}

	void buildCalibrateArgs_bothProjectorVersions_mapCorrectly()
	{
		for (int version : { 3010, 4710 })
		{
			QStringList args = XemaCameraWindow::buildCalibrateArgs(
				"C:/calib/XEMA/cam1", version, 20, XemaCalibMode::Color, "C:/calib/XEMA/cam1/param.txt");
			int version_idx = args.indexOf("--version");
			QCOMPARE(args.value(version_idx + 1), QString::number(version));
		}
	}

	void buildCalibrateArgs_unknownProjectorVersion_passesThroughAsIs()
	{
		// Documents current behavior: buildCalibrateArgs does NOT validate projector_version --
		// a 0 (never connected this session, nothing saved in config.json) is passed straight
		// through as "--version 0". Whether calibration.exe fails clearly or confusingly on
		// that is calibration.exe's problem, not this function's -- but flagging it here means
		// a future decision to validate earlier (e.g. in onCalibrateClicked) has a test to
		// update instead of silently changing unnoticed behavior.
		QStringList args = XemaCameraWindow::buildCalibrateArgs(
			"C:/calib/XEMA/cam1", 0, 20, XemaCalibMode::Color, "C:/calib/XEMA/cam1/param.txt");
		int version_idx = args.indexOf("--version");
		QCOMPARE(args.value(version_idx + 1), QString("0"));
	}

	void buildCalibrateArgs_doesNotDeleteOrTouchFilesystem()
	{
		// buildCalibrateArgs is documented as pure (no side effects) -- confirm calling it
		// doesn't create calib_out_path, unlike calibrateThreadFunc's stale-file-delete step
		// which runs separately, outside this function.
		QString out_path = QDir::tempPath() + "/xema_test_buildCalibrateArgs_no_touch.txt";
		QFile::remove(out_path); // ensure clean starting state
		XemaCameraWindow::buildCalibrateArgs("C:/calib/XEMA/cam1", 3010, 20, XemaCalibMode::Color, out_path);
		QVERIFY(!QFile::exists(out_path));
	}

	// ---------------- buildCorrectArgs ----------------

	void buildCorrectArgs_hasSeparateInAndOutParamPaths()
	{
		QStringList args = XemaCameraWindow::buildCorrectArgs(
			"C:/calib/XEMA/cam1", 3010, 20, XemaCalibMode::Color,
			"C:/staging/param.txt", "C:/calib/XEMA/cam1/param.txt");

		int in_idx = args.indexOf("--param-in");
		int out_idx = args.indexOf("--param-out");
		QVERIFY(in_idx >= 0);
		QVERIFY(out_idx >= 0);
		QCOMPARE(args.value(in_idx + 1), QDir::toNativeSeparators(QString("C:/staging/param.txt")));
		QCOMPARE(args.value(out_idx + 1), QDir::toNativeSeparators(QString("C:/calib/XEMA/cam1/param.txt")));
		// Confirms the two paths are genuinely independent, not the same value duplicated --
		// mixing these up would silently corrupt calibration data.
		QVERIFY(args.value(in_idx + 1) != args.value(out_idx + 1));
	}

	void buildCorrectArgs_usesCorrectNotCalibrateFlag()
	{
		QStringList args = XemaCameraWindow::buildCorrectArgs(
			"C:/calib/XEMA/cam1", 3010, 20, XemaCalibMode::Color, "in.txt", "out.txt");
		QVERIFY(args.contains("--correct"));
		QVERIFY(!args.contains("--calibrate"));
	}

	void buildCorrectArgs_monoMode_usesPatterns()
	{
		QStringList args = XemaCameraWindow::buildCorrectArgs(
			"C:/calib/XEMA/cam1", 3010, 20, XemaCalibMode::Mono, "in.txt", "out.txt");
		int use_idx = args.indexOf("--use");
		QCOMPARE(args.value(use_idx + 1), QString("patterns"));
	}

	// ---------------- calibModeFromConfig ----------------

	void calibModeFromConfig_missingKey_defaultsToColor()
	{
		// Simulates loading a config.json saved before the color/mono toggle existed.
		QJsonObject old_config;
		old_config["board_spacing_mm"] = 20;
		old_config["identity"] = "cam1";
		QCOMPARE((int)XemaCameraWindow::calibModeFromConfig(old_config), (int)XemaCalibMode::Color);
	}

	void calibModeFromConfig_explicitMono_returnsMono()
	{
		QJsonObject obj;
		obj["calib_mode"] = "mono";
		QCOMPARE((int)XemaCameraWindow::calibModeFromConfig(obj), (int)XemaCalibMode::Mono);
	}

	void calibModeFromConfig_explicitColor_returnsColor()
	{
		QJsonObject obj;
		obj["calib_mode"] = "color";
		QCOMPARE((int)XemaCameraWindow::calibModeFromConfig(obj), (int)XemaCalibMode::Color);
	}

	void calibModeFromConfig_garbageValue_fallsBackToColor()
	{
		QJsonObject obj;
		obj["calib_mode"] = "banana";
		QCOMPARE((int)XemaCameraWindow::calibModeFromConfig(obj), (int)XemaCalibMode::Color);
	}

	// ---------------- boardSpacingFromConfig ----------------

	void boardSpacingFromConfig_missingKey_defaultsTo80()
	{
		QJsonObject obj;
		QCOMPARE((int)XemaCameraWindow::boardSpacingFromConfig(obj), (int)XemaBoardSpacingMm::Spacing80);
	}

	void boardSpacingFromConfig_allValidValues_mapCorrectly()
	{
		QJsonObject obj4; obj4["board_spacing_mm"] = 4;
		QCOMPARE((int)XemaCameraWindow::boardSpacingFromConfig(obj4), (int)XemaBoardSpacingMm::Spacing4);

		QJsonObject obj12; obj12["board_spacing_mm"] = 12;
		QCOMPARE((int)XemaCameraWindow::boardSpacingFromConfig(obj12), (int)XemaBoardSpacingMm::Spacing12);

		QJsonObject obj20; obj20["board_spacing_mm"] = 20;
		QCOMPARE((int)XemaCameraWindow::boardSpacingFromConfig(obj20), (int)XemaBoardSpacingMm::Spacing20);

		QJsonObject obj40; obj40["board_spacing_mm"] = 40;
		QCOMPARE((int)XemaCameraWindow::boardSpacingFromConfig(obj40), (int)XemaBoardSpacingMm::Spacing40);

		QJsonObject obj80; obj80["board_spacing_mm"] = 80;
		QCOMPARE((int)XemaCameraWindow::boardSpacingFromConfig(obj80), (int)XemaBoardSpacingMm::Spacing80);
	}

	void boardSpacingFromConfig_garbageValue_fallsBackTo80()
	{
		// A hand-edited or corrupted config.json with an invalid spacing (999mm doesn't exist
		// as a board option) must not produce an uninitialized/undefined enum value.
		QJsonObject obj;
		obj["board_spacing_mm"] = 999;
		QCOMPARE((int)XemaCameraWindow::boardSpacingFromConfig(obj), (int)XemaBoardSpacingMm::Spacing80);
	}

	void boardSpacingFromConfig_negativeValue_fallsBackTo80()
	{
		QJsonObject obj;
		obj["board_spacing_mm"] = -20;
		QCOMPARE((int)XemaCameraWindow::boardSpacingFromConfig(obj), (int)XemaBoardSpacingMm::Spacing80);
	}
};

QTEST_MAIN(TestXemaCalibLogic)
#include "tst_xema_calib_logic.moc"
