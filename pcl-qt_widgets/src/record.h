#pragma once

#include <QObject>

#include <iostream>
#include <string>

class Record : public QObject
{
	Q_OBJECT

public:
	Record();
	virtual ~Record();

signals:
	void progressBarUpdate(int);
	void infoRec(QString);
};

