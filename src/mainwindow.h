#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QThread>
#include<QMutex>
#include"getRosData.h"
#include"qevent.h"
#include"algorithm.h"
#include <QMetaType>

#define PER_LINER  0.6
#define PER_ANGLER  M_PI/10

namespace Ui {
class MainWindow;
}

class CKinectListenerThread : public QThread{
    Q_OBJECT
public:
    CKinectListenerThread(int argc, char** argv):
    argc_(argc),
    argv_(argv),
    kl_(nullptr)
    {
        qRegisterMetaType<sensor_msgs::CameraInfo>("sensor_msgs::CameraInfo"); 
    };
    ~CKinectListenerThread()
    {
         ros::shutdown();
         if(kl_!=nullptr)
            delete kl_;
    }
protected:
    virtual void run();
private:
    int argc_;
    char** argv_;
    CKinectListener* kl_;
public slots:
    void slotCvImage(const QImage& );
    void slotCvImageDepth(const QImage& );
    void slotCarRun(const float&,const float&);
    void slotCameraInfo(const sensor_msgs::CameraInfo&);
signals:
    void sigCvImage(const QImage& );
    void sigCvImageDepth(const QImage& );
    void sigCameraInfo(const sensor_msgs::CameraInfo&);
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();
protected:
    virtual void keyPressEvent(QKeyEvent *ev);
    virtual void keyReleaseEvent(QKeyEvent *ev);
public slots:
    void slotCvImageRGB(const QImage&) ;
    void slotCvImageDepth(const QImage&) ;
    void slotCameraInfo(const sensor_msgs::CameraInfo&);
signals:
    void sigCarRun(const float&,const float&);
 private:
    Ui::MainWindow *ui_;
    Sn3DAlgorithmRebuild sn3dRebuild;
    CKinectListenerThread *kLThread_;
    QMutex mutex1_;
    QMutex mutex2_;
    bool   isUpKeyDown_, isDownKeyDown_, isLeftKeyDown_, isRightKeyDown_;
};

#endif // MAINWINDOW_H
