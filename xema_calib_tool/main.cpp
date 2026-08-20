#include <QApplication>
#include "XemaCameraWindow.h"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	XemaCameraWindow window;
	window.show();

	return app.exec();
}
