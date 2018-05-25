#include "mainwindow.h"
#include "ui_mainwindow.h"


#include <QFileDialog>
#include <QDir>
#include <QThread>
#include <QDateTime>
#include <QTextStream>
#include <QMessageBox>

//bool next_iteration = false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(onModelFileOpenSlot()));
	connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(onDataFileOpenSlot()));
	connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(onVisualizerOpenSlot()));
	connect(ui->pushButton_4, SIGNAL(clicked()), this, SLOT(onPreviewOpenSlot())); 
	connect(ui->pushButton_5, SIGNAL(clicked()), this, SLOT(onComputeKdtreeSlot()));
	connect(ui->pushButton_6, SIGNAL(clicked()), this, SLOT(onComputeCentroidSlot()));
	connect(ui->pushButton_7, SIGNAL(clicked()), this, SLOT(onComputeNormalsSlot()));
	connect(ui->pushButton_8, SIGNAL(clicked()), this, SLOT(onComputeFPFHSlot()));
	connect(ui->pushButton_9, SIGNAL(clicked()), this, SLOT(onFilterNDownsampling()));
	connect(ui->pushButton_10, SIGNAL(clicked()), this, SLOT(onComputeEGI()));
	connect(ui->pushButton_11, SIGNAL(clicked()), this, SLOT(onRegistration()));
	connect(ui->pushButton_12, SIGNAL(clicked()), this, SLOT(onInitialAlignment()));
	connect(ui->pushButton_13, SIGNAL(clicked()), this, SLOT(onSACSegment()));

	ui->progressBar->setRange(0, 100);
	ui->progressBar->setValue(0);

	ui->pushButton_5->setEnabled(false);
	ui->pushButton_6->setEnabled(false);
	ui->pushButton_7->setEnabled(false);
	ui->pushButton_8->setEnabled(false);
	ui->pushButton_9->setEnabled(false);
	ui->pushButton_10->setEnabled(false);
	ui->pushButton_11->setEnabled(false);
	ui->pushButton_12->setEnabled(false);
	ui->pushButton_13->setEnabled(false);

	ui->spinBox_2->setValue(100);
	ui->doubleSpinBox->setValue(20.0);
	ui->doubleSpinBox_2->setValue(30.0);

	QStatusBar* bar = ui->statusBar;
	statusLabel = new QLabel;
	statusLabel->setMinimumSize(500, 20);
	statusLabel->setFrameShape(QFrame::Panel);
	statusLabel->setFrameShadow(QFrame::Sunken);
	bar->addWidget(statusLabel);
	statusLabel->setText("hello pcl!");

	//Default filename
	fileName_Model = "E:/Projects/pcl/pcl-qt_widgets/build/data/blade_surface_model.ply";
	fileName_Data = "E:/Projects/pcl/pcl-qt_widgets/build/data/blade_surface_data.ply";
	ui->textBrowser->setText("blade_model.ply");
	ui->textBrowser_2->setText("blade_data.ply");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void
MainWindow::icp_proceed(QString filename_model, QString filename_data, int iterations)
{
	icpreg = new ICPReg(filename_model, filename_data, iterations);//你的工作线程类
	QThread * th = new QThread();// 创建一个线程
	icpreg->moveToThread(th);//把你的类放在刚创建的线程里

	//连接MainWindow主界面类和工作线程类的信号槽
	connect(th, SIGNAL(started()), icpreg, SLOT(OnStarted()));//线程开始信号
	connect(icpreg, SIGNAL(finished()), this, SLOT(onViewerOff()));
	connect(icpreg, SIGNAL(finished()), th, SLOT(quit()));//线程结束信号
	connect(icpreg->record, SIGNAL(progressBarUpdate(int)), this, SLOT(onProgressBarUpdateSlot(int)));
	connect(icpreg->record, SIGNAL(infoUpdate(QString)), this, SLOT(onInfoRecSlot(QString)));
	connect(icpreg->record, SIGNAL(statusUpdate(QString)), this, SLOT(onStatusUpdateSlot(QString)));

	//线程开始信号
	th->start();
}

void
MainWindow::onModelFileOpenSlot()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PLY/PCD files(*.ply *.pcd)"));
	QFileInfo temDir(fileName);
	if (!fileName.isEmpty())
	{
		ui->textBrowser->setText(temDir.fileName());
		fileName_Model = fileName;
	}
}

void
MainWindow::onDataFileOpenSlot()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PLY/PCD files(*.ply *.pcd)"));
	ui->textBrowser_2->setText("suixingdianban_filtered.ply");
	QFileInfo temDir(fileName);
	if (!fileName.isEmpty())
	{
		ui->textBrowser_2->setText(temDir.fileName());
		fileName_Data = fileName;
	}
}

void
MainWindow::onVisualizerOpenSlot()
{
	if ((!fileName_Model.isEmpty()) && (!fileName_Model.isEmpty()))
	{
		ui->pushButton_3->setEnabled(false);
		iterations = ui->spinBox->value();
		icp_proceed(fileName_Model, fileName_Data, iterations);
	}
}

void
MainWindow::onProgressBarUpdateSlot(int percent)
{
	ui->progressBar->setValue(percent);
}

void
MainWindow::onInfoRecSlot(QString content)
{
	ui->textBrowser_3->setText(content);
	ui->textBrowser_3->moveCursor(QTextCursor::End);
}

void
MainWindow::onStatusUpdateSlot(QString info)
{
	statusLabel->setText(info);
}

void
MainWindow::onViewerOff()
{
	ui->pushButton_3->setEnabled(true);

	ui->pushButton_4->setEnabled(true);

	ui->pushButton_5->setEnabled(false);

	ui->pushButton_6->setEnabled(false);

	ui->pushButton_7->setEnabled(false);

	ui->pushButton_8->setEnabled(false);

	ui->pushButton_9->setEnabled(false);

	ui->pushButton_10->setEnabled(false);

	ui->pushButton_11->setEnabled(false);

	ui->pushButton_12->setEnabled(false);

	ui->pushButton_13->setEnabled(false);

}

void
MainWindow::onPreviewOpenSlot()
{
	if (!fileName_Model.isEmpty())
	{
		ui->pushButton_4->setEnabled(false);
		ui->pushButton_5->setEnabled(true);
		ui->pushButton_6->setEnabled(true);
		ui->pushButton_7->setEnabled(true);
		ui->pushButton_8->setEnabled(true);
		ui->pushButton_9->setEnabled(true);
		ui->pushButton_10->setEnabled(true);
		ui->pushButton_11->setEnabled(true);
		ui->pushButton_12->setEnabled(true);
		ui->pushButton_13->setEnabled(true);
	}
		
	visualization = new Visualization(fileName_Model, fileName_Data);
	QThread * th = new QThread();	
	visualization->moveToThread(th);

	connect(th, SIGNAL(started()), visualization, SLOT(OnStarted()));
	connect(visualization, SIGNAL(finished()), this, SLOT(onViewerOff()));
	connect(visualization, SIGNAL(finished()), th, SLOT(quit()));
	connect(visualization->record, SIGNAL(progressBarUpdate(int)), this, SLOT(onProgressBarUpdateSlot(int)));
	connect(visualization->record, SIGNAL(infoUpdate(QString)), this, SLOT(onInfoRecSlot(QString)));
	connect(visualization->record, SIGNAL(statusUpdate(QString)), this, SLOT(onStatusUpdateSlot(QString)));

	th->start();
}

void
MainWindow::onComputeKdtreeSlot()
{
	visualization->feature_id = 1;
}

void
MainWindow::onComputeCentroidSlot()
{
	visualization->feature_id = 2;
}

void
MainWindow::onComputeNormalsSlot()
{
	visualization->normal_level = ui->spinBox_2->value();
	visualization->normal_scale = ui->doubleSpinBox->value();
	visualization->search_radius = ui->doubleSpinBox_2->value();
	visualization->feature_id = 3;
}

void
MainWindow::onComputeFPFHSlot()
{
	visualization->search_radius = ui->doubleSpinBox_2->value();
	visualization->feature_id = 4;
}

void
MainWindow::onFilterNDownsampling()
{
	visualization->feature_id = 5;
}

void
MainWindow::onComputeEGI()
{
	visualization->feature_id = 6;
}

void
MainWindow::onRegistration()
{
	visualization->rotation.at(0) = ui->doubleSpinBox_3->value();
	visualization->rotation.at(1) = ui->doubleSpinBox_4->value();
	visualization->rotation.at(2) = ui->doubleSpinBox_5->value();
	visualization->feature_id = 7;
}

void
MainWindow::onInitialAlignment()
{
	visualization->feature_id = 8;
}

void
MainWindow::onSACSegment()
{
	visualization->feature_id = 9;
}