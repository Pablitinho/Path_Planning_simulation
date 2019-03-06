#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "vehicle.h"
#include "ptg.h"

void MainWindow::ComputeTrajectory()
{
	std::vector<float> start_vector {0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	std::vector<vehicle> predictions;
	
	int target = 0;
	std::vector<float> delta{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	std::vector<float> start_s{ 10.0f, 10.0f, 0.0f};
	std::vector<float> start_d{ 4.0f, 0.0f, 0.0f };
	float time = 5.0f;

	vehicle * my_vehicle = new vehicle(start_vector);
	predictions.push_back(*my_vehicle);
	
	ptg poly_traj_gen;

	trajectory_t best_traj = poly_traj_gen.Find_Trajectory(start_s, start_d, target, delta, time, predictions);
}
//---------------------------------------------------------------------------
void MainWindow::Set_Data(QwtPlot * plot,std::string title,std::vector<QPointF> vect_points, QColor color)
{

	// Compute the trayectory of the car
	ComputeTrajectory();

	QwtPlotCurve *curve = new QwtPlotCurve();
	curve->setTitle(title.c_str());
	curve->setPen(color, 4),
		curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);

	QwtSymbol *symbol = new QwtSymbol(QwtSymbol::Ellipse,
		QBrush(Qt::yellow), QPen(Qt::red, 2), QSize(8, 8));
	curve->setSymbol(symbol);

	QPolygonF points;

	/* Copy to the properly structure */
	for (int32_t id=0; id < vect_points.size(); id++) 
	{
		points.push_back(vect_points[id]);
	}

	curve->setSamples(points);
	curve->attach(plot);
}
//---------------------------------------------------------------------------
void MainWindow::Create_plot()
{
	plot = new QwtPlot(this);

	//QwtPlot plot;
	plot->setTitle("Trajectory");
	plot->setCanvasBackground(Qt::white);
	plot->setAxisScale(QwtPlot::yLeft, -10.0, 10.0);
	plot->insertLegend(new QwtLegend());

	QwtPlotGrid *grid = new QwtPlotGrid();
	grid->attach(plot);

	std::vector<QPointF> vect_vehicle;

	vect_vehicle.push_back(QPointF(0.0, 4.4));
	vect_vehicle.push_back(QPointF(1.0, 3.0));
	vect_vehicle.push_back(QPointF(2.0, 4.5));
	vect_vehicle.push_back(QPointF(3.0, 6.8));
	vect_vehicle.push_back(QPointF(-4.0, 7.9));
	vect_vehicle.push_back(QPointF(5.0, -7.1));

	Set_Data(plot,"Vehicle", vect_vehicle,Qt::red);

	setCentralWidget(plot);
}
//---------------------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	Create_plot();	
}
//---------------------------------------------------------------------------
MainWindow::~MainWindow()
{

    delete ui;
}
//---------------------------------------------------------------------------
