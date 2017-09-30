#pragma once

#include <QObject>
#include <QFileDialog>
#include <QDir>
#include <QThread>
#include <QDateTime>
#include <QTextStream>
#include <QMessageBox>

#include <iostream>
#include <string>

class Record : public QObject
{
	Q_OBJECT

public:
	Record();
	virtual ~Record();

	QString info;

	void infoRec(QString);

signals:
	void progressBarUpdate(int);
	void infoUpdate(QString content);
	void statusUpdate(QString info);
};

