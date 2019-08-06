#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "getRosData.h"
#include<geometry_msgs/Twist.h>

void CKinectListenerThread::slotCvImage(const QImage& qImage)
{
    emit sigCvImage(qImage);
}


void CKinectListenerThread::slotCvImageDepth(const QImage& qImage)
{
    emit sigCvImageDepth(qImage);
}

void CKinectListenerThread::slotCarRun(const float& liner,const float& angler)
{
   if(kl_ == nullptr)
        return;
    geometry_msgs::Twist speed;
    speed = geometry_msgs::Twist();
    speed.linear.x=liner;
    speed.angular.z=angler;  
    kl_->runPub.publish(speed);
} 

void CKinectListenerThread::run(){
    ros::init(argc_,argv_,"my_Kinect_listener");
    kl_ = new  CKinectListener();
    ros::AsyncSpinner spinner(8); // Use8 threads
    connect(kl_,SIGNAL(sigCvImage(const QImage&)),this,SLOT(slotCvImage(const QImage&)));
    connect(kl_,SIGNAL(sigCvImageDepth(const QImage&)),this,SLOT(slotCvImageDepth(const QImage&)));
    spinner.start();
    ros::waitForShutdown();       
}

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow())
{
    ui->setupUi(this);
    kLThread = new CKinectListenerThread(argc,argv);
    connect(kLThread,SIGNAL(sigCvImage(const QImage&)),this,SLOT(slotCvImageRGB(const QImage&)));
    connect(kLThread,SIGNAL(sigCvImageDepth(const QImage&)),this,SLOT(slotCvImageDepth(const QImage&)));
    connect(this, SIGNAL(sigCarRun(const float&,const float&)),kLThread,SLOT(slotCarRun(const float&,const float&)));
    kLThread->start();
    this->grabKeyboard();
    isUpKeyDown = isDownKeyDown = isLeftKeyDown = isRightKeyDown = false;
    rosLinear = rosAngular = 0;
}

MainWindow::~MainWindow()
{
    kLThread->terminate();
    delete ui;
    delete kLThread;
}

void MainWindow::slotCvImageRGB(const QImage& qImage)
{
   mutex1.lock();
    ui->rgbImageLabel->setPixmap(QPixmap::fromImage(qImage));
    mutex1.unlock();
}

void MainWindow::slotCvImageDepth(const QImage& qImage)
{
    mutex2.lock();
    ui->depthImageLabel->setPixmap(QPixmap::fromImage(qImage));
    mutex2.unlock();
}

void MainWindow::keyPressEvent(QKeyEvent *ev)
{
    if(ev ->key()== Qt::Key_Up)
    {
        isUpKeyDown = true;
        if(isDownKeyDown == true)
            rosLinear = 0;
        else
            rosLinear = PER_LINER;
    }
    else if(ev ->key()== Qt::Key_Down)
    {
        isDownKeyDown = true;
        if(isUpKeyDown == true)
            rosLinear = 0;
        else
            rosLinear = -PER_LINER;
    }
    else if(ev ->key()== Qt::Key_Left)
    {
        isLeftKeyDown = true;
        if(isRightKeyDown)
            rosAngular = 0;
        else
            rosAngular = PER_ANGLER;
    }
    else if(ev ->key()== Qt::Key_Right)
    {
        isRightKeyDown = true;
        if(isLeftKeyDown)
            rosAngular = 0;
        else
            rosAngular = -PER_ANGLER;
    }
    emit sigCarRun(rosLinear,rosAngular);
}

void MainWindow::keyReleaseEvent(QKeyEvent *ev)
{
    if(ev ->key()== Qt::Key_Up)
    {
        isUpKeyDown = false;
        if(isDownKeyDown)
            rosLinear = -PER_LINER;
        else
            rosLinear = 0;
    }
    else if(ev ->key()== Qt::Key_Down)
    {
        isDownKeyDown = false;
          if(isUpKeyDown)
            rosLinear =PER_LINER;
        else
            rosLinear = 0;
    }
    else if(ev ->key()== Qt::Key_Left)
    {
        isLeftKeyDown = false;
        if(isRightKeyDown)
            rosAngular =  -PER_ANGLER;
        else
            rosAngular = 0;
    }
    else if(ev ->key()== Qt::Key_Right)
    {
        isRightKeyDown = false;
           if(isLeftKeyDown)
            rosAngular =  PER_ANGLER;
        else
            rosAngular = 0;
    }
    emit sigCarRun(rosLinear,rosAngular);
}