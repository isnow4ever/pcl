#include "ProjectMainWindow.h"
#include <QTextCodec>

//程序发布时，可以不要注释以下语句，编译的时候就不会带控制台输出窗口
//#pragma comment( linker, "/subsystem:windows /entry:mainCRTStartup" )

int main( int argc, char **argv ) 
{
	QApplication *app = new QApplication(argc, argv);
	QTextCodec::setCodecForLocale(QTextCodec::codecForName("GB2312"));

	ProjectMainWindow *window = new ProjectMainWindow();
	window->show();
	return app->exec();
};
