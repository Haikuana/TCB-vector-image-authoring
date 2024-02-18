#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QtWidgets/QStyleFactory>
#include "Corefuncs/ImageApproximation.h"

int main(int argc, char *argv[])
{
#if 0
	Image_Approximation appimage;
	QString infname("E:/Projects/ImageApproxiTCB/ImageApproxiEdit/build/bin/Results/authoring/test1.author");
	CMInfo realCm;
	appimage.gene_control_mesh(infname, realCm);
#endif // 0

#if 1
	QApplication a(argc, argv);
	QFont font = a.font();
	font.setPointSize(10);
	a.setFont(font);
	QApplication::setStyle("fusion");
	EDITGUI::MainWindow w;
	w.show();
	return a.exec();
#endif // 0

}
