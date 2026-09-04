// tst_smoke.cpp
//
// Bisection step: does QtTest itself work in this build at all, with NO XemaCameraWindow
// construction? If this crashes too, the problem is in CMake/Qt/linkage, not in
// XemaCameraWindow's constructor. If this PASSES cleanly, the problem is specifically
// something XemaCameraWindow's constructor does.
//
// Temporary build wiring (add to CMakeLists.txt right next to tst_xema_click_races, or just
// swap the SOURCES of that target to tst_smoke.cpp for one build/run):
//
//   add_executable(tst_smoke tst_smoke.cpp)
//   target_link_libraries(tst_smoke PRIVATE Qt5::Widgets Qt5::Test)

#include <QtTest/QtTest>

class TestSmoke : public QObject
{
	Q_OBJECT

private slots:
	void trivialPass()
	{
		QVERIFY(true);
	}

	void constructPlainQWidget()
	{
		// A bare QWidget, not XemaCameraWindow -- confirms Qt Widgets itself works in this
		// process (creates a QApplication implicitly via QTEST_MAIN below).
		QWidget w;
		QVERIFY(!w.isVisible()); // never shown, so this should just be false, not a crash
	}
};

QTEST_MAIN(TestSmoke)
#include "tst_smoke.moc"
