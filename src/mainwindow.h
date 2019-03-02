#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qwt_plot.h"
#include <qwt_plot_canvas.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
	void Create_plot();
	void Set_Data(QwtPlot * plot, std::string title, std::vector<QPointF> vect_points, QColor color);
    Ui::MainWindow *ui;

	void ComputeTrajectory();

	QwtPlot * plot;
};

#endif // MAINWINDOW_H
