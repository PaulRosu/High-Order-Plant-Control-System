#include "mainwindow.h"

#include <QApplication>
#include <QDebug>

#if defined(Q_OS_WIN)
#include <windows.h>
#include <stdio.h>
#endif

void setupDebugOutput()
{
#if defined(Q_OS_WIN)
    // If running from console, attach to it
    if (AttachConsole(ATTACH_PARENT_PROCESS)) {
        freopen("CONOUT$", "w", stdout);
        freopen("CONOUT$", "w", stderr);
    }
#endif
}

int main(int argc, char *argv[])
{
    setupDebugOutput();
    
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    
    return a.exec();
}
