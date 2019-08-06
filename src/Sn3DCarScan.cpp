#include "Sn3DCarScan.h"
#include "mainwindow.h"
#include <QApplication>
#include<iostream>
#include<ros/ros.h>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w(argc,argv);
    w.show();
    return a.exec();
}