#include "mainwindow.h"
#include <QApplication>
#include "qwt_plot.h"

QwtPlot * plot;

int main(int argc, char *argv[])
{
	//argc = 1;
    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    return a.exec();
}
