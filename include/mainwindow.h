#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QThread>
#include<QMutex>
#include<QTimer>
#include"qevent.h"
#include <QMetaType>
#include"kinectListenerThread.h"
#include "algorithmThread.h"
#include "vulkanwindow.h"

#define PER_LINER  0.6
#define PER_ANGLER  M_PI/10

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(int argc, char** argv, VulkanWindow*  vulkanWindow,QWidget *parent = nullptr);
    ~MainWindow();
protected:
    virtual void keyPressEvent(QKeyEvent *ev);
    virtual void keyReleaseEvent(QKeyEvent *ev);
public slots:
    void slotCvImageRGB(const cv_bridge::CvImagePtr) ;
    void slotCvImageDepth(const cv_bridge::CvImagePtr) ;
    void slotTestBtnClicked();
    void slotFinishInitScan();
    void slotInitScanMoving();
signals:
    void sigCarRun(const float&,const float&);
    void sigInitScan();
 private:
    Ui::MainWindow *ui_;
    CKinectListenerThread *kLThread_;
    CAlgorithimThread         *aThread_;
    QMutex mutex1_;
    QMutex mutex2_;
    bool   isUpKeyDown_, isDownKeyDown_, isLeftKeyDown_, isRightKeyDown_;
    QVector<QRgb> colorTable_;
};

#endif // MAINWINDOW_H
