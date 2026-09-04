// tst_window_construct.cpp
//
// Narrower bisection than tst_xema_click_races.cpp: does XemaCameraWindow even construct
// successfully, with NO test hooks, NO QTemporaryDir, NO invokeMethod calls -- just the bare
// constructor? If this crashes, the problem is inside the constructor itself (or something it
// calls, like initStatusConsole()/AllocConsole(), loadConfig(), or the SDK DLL load). If this
// PASSES, the problem is something specific to tst_xema_click_races.cpp's setup.
//
// Build wiring (temporary, same pattern as xema_camera_gui/tst_xema_click_races -- copy their
// target_link_libraries/target_include_directories/XEMA_TEST_HOOKS/DLL-copy blocks verbatim,
// just swapping the executable name and source file):
//
//   add_executable(tst_window_construct tst_window_construct.cpp XemaCameraWindow.cpp XemaCameraWindow.h)
//   target_compile_definitions(tst_window_construct PRIVATE XEMA_TEST_HOOKS)
//   target_link_libraries(tst_window_construct PRIVATE Qt5::Widgets Qt5::Test ${OpenCV_LIBS} open_cam3d_sdk)
//   target_include_directories(tst_window_construct PRIVATE ${OpenCV_INCLUDE_DIRS} ${XEMA_SDK_DIR})
//   add_custom_command(TARGET tst_window_construct POST_BUILD
//       COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:open_cam3d_sdk> $<TARGET_FILE_DIR:tst_window_construct>)
//   if(MSVC)
//       target_compile_options(tst_window_construct PRIVATE /utf-8)
//   endif()

#include <QtTest/QtTest>
#include "XemaCameraWindow.h"

class TestWindowConstruct : public QObject
{
	Q_OBJECT

private slots:
	void constructOnly()
	{
		qDebug() << "about to construct XemaCameraWindow";
		XemaCameraWindow w;
		qDebug() << "constructed XemaCameraWindow successfully";
		QVERIFY(true);
	}
};

QTEST_MAIN(TestWindowConstruct)
#include "tst_window_construct.moc"
