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

	ui->progressBar->setRange(0, 100);
	ui->progressBar->setValue(0);
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
	connect(icpreg, SIGNAL(finished()), th, SLOT(terminate()));//线程结束信号
	connect(icpreg, SIGNAL(progressBarUpdate(int)), this, SLOT(onProgressBarUpdateSlot(int)));
	connect(icpreg, SIGNAL(infoRec(QString)), this, SLOT(onInfoRecSlot(QString)));

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
MainWindow::onInfoRecSlot(QString info)
{
	QFile file("file.txt");
	if (!file.open(QFile::ReadWrite | QFile::Text))
		QMessageBox::warning(this, "sdf", "can't open", QMessageBox::Yes);
	QTextStream in(&file);
	QString content = in.readAll();
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString str = time.toString("yyyy-MM-dd hh:mm:ss"); //设置显示格式
	in << str << endl;
	in << info << endl;
	file.close();

	content.append(str + "\r\n");
	content.append(info + "\r\n");
	
	ui->textBrowser_3->setText(content);
	ui->textBrowser_3->moveCursor(QTextCursor::End);
}

void
MainWindow::onViewerOff()
{
	ui->pushButton_3->setEnabled(true);

	ui->pushButton_4->setEnabled(true);
}

void
MainWindow::onPreviewOpenSlot()
{
	if (!fileName_Model.isEmpty())
		ui->pushButton_4->setEnabled(false);

	visualization = new Visualization(fileName_Model);
	QThread * th = new QThread();
	visualization->moveToThread(th);

	connect(th, SIGNAL(started()), visualization, SLOT(OnStarted()));
	connect(visualization, SIGNAL(finished()), this, SLOT(onViewerOff()));
	connect(visualization, SIGNAL(finished()), th, SLOT(terminate()));

	th->start();
}