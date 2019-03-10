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

#include "vehicle.h"
#include "ptg.h"

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
	void Create_plot(std::vector<point_2d_t> points);
	void Set_Data(QwtPlot * plot, std::string title, std::vector<point_2d_t> vect_points, QColor color);
    Ui::MainWindow *ui;

	std::vector<point_2d_t> ComputeTrajectory();

	QwtPlot * plot;
};

#endif // MAINWINDOW_H
