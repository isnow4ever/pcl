#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ICPReg.h"
#include "visualization.h"
#include <QMainWindow>

#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

	ICPReg *icpreg;
	Visualization *visualization;

	void icp_proceed(QString filename_model, QString filename_data, int iterations);

private:
    Ui::MainWindow *ui;

	QString fileName_Model;
	QString fileName_Data;

	int iterations;

private slots:
	void onVisualizerOpenSlot();
	void onModelFileOpenSlot();
	void onDataFileOpenSlot();
	void onProgressBarUpdateSlot(int);
	void onInfoRecSlot(QString);
	void onViewerOff();

	void onPreviewOpenSlot();
	void onComputeKdtreeSlot();
	void onComputeCentroidSlot();
};

#endif // MAINWINDOW_H
