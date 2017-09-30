#include "record.h"



Record::Record()
{
}


Record::~Record()
{
}

void
Record::infoRec(QString info)
{
	QFile file("file.txt");
	if (!file.open(QFile::ReadWrite | QFile::Text))
		emit statusUpdate("can't open file.txt");
	QTextStream in(&file);
	QString content = in.readAll();
	QDateTime time = QDateTime::currentDateTime();//��ȡϵͳ���ڵ�ʱ��
	QString str = time.toString("yyyy-MM-dd hh:mm:ss"); //������ʾ��ʽ
	in << str << endl;
	in << info << endl;
	file.close();

	content.append(str + "\r\n");
	content.append(info + "\r\n");

	emit infoUpdate(content);
}