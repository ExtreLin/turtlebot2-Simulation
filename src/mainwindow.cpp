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
    kl_->runPub_.publish(speed);
} 

void CKinectListenerThread::slotCameraInfo(const sensor_msgs::CameraInfo& msg)
{
    emit sigCameraInfo(msg);
}

void CKinectListenerThread::run(){
    ros::init(argc_,argv_,"my_Kinect_listener");
    kl_ = new  CKinectListener();
    ros::AsyncSpinner spinner(8); // Use8 threads
    connect(kl_,SIGNAL(sigCvImage(const QImage&)),this,SLOT(slotCvImage(const QImage&)));
    connect(kl_,SIGNAL(sigCvImageDepth(const QImage&)),this,SLOT(slotCvImageDepth(const QImage&)));
    connect(kl_,SIGNAL(sigCameraInfo(const sensor_msgs::CameraInfo&)),this,SLOT(slotCameraInfo(const sensor_msgs::CameraInfo&)));
    spinner.start();
    ros::waitForShutdown();       
}

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::MainWindow()),
    sn3dRebuild(),
    kLThread_(nullptr),
    mutex1_(),
    mutex2_(),
    isUpKeyDown_(false),
    isDownKeyDown_(false),
    isLeftKeyDown_(false),
    isRightKeyDown_(false)
{
    ui_->setupUi(this);
    qRegisterMetaType<sensor_msgs::CameraInfo>("sensor_msgs::CameraInfo"); 
    kLThread_ = new CKinectListenerThread(argc,argv);
    connect(kLThread_,SIGNAL(sigCvImage(const QImage&)),this,SLOT(slotCvImageRGB(const QImage&)));
    connect(kLThread_,SIGNAL(sigCvImageDepth(const QImage&)),this,SLOT(slotCvImageDepth(const QImage&)));
    connect(kLThread_,SIGNAL(sigCameraInfo(const sensor_msgs::CameraInfo&)),this,SLOT(slotCameraInfo(const sensor_msgs::CameraInfo&)));
    connect(this, SIGNAL(sigCarRun(const float&,const float&)),kLThread_,SLOT(slotCarRun(const float&,const float&)));
    kLThread_->start();
    this->grabKeyboard();
}

MainWindow::~MainWindow()
{
    kLThread_->terminate();
    delete ui_;
    delete kLThread_;
}

void MainWindow::slotCvImageRGB(const QImage& qImage)
{
   mutex1_.lock();
    ui_->rgbImageLabel->setPixmap(QPixmap::fromImage(qImage));
    mutex1_.unlock();
}

void MainWindow::slotCvImageDepth(const QImage& qImage)
{
    mutex2_.lock();
    ui_->depthImageLabel->setPixmap(QPixmap::fromImage(qImage));
    mutex2_.unlock();
}

void MainWindow::slotCameraInfo(const sensor_msgs::CameraInfo&msg)
{
    sn3dRebuild.setCameraInfo(msg);
}

void MainWindow::keyPressEvent(QKeyEvent *ev)
{
    float rosLinear,rosAngular;
    if(ev ->key()== Qt::Key_Up)
    {
        isUpKeyDown_ = true;
        if(isDownKeyDown_ == true)
            rosLinear = 0;
        else
            rosLinear = PER_LINER;
    }
    else if(ev ->key()== Qt::Key_Down)
    {
        isDownKeyDown_ = true;
        if(isUpKeyDown_ == true)
            rosLinear = 0;
        else
            rosLinear = -PER_LINER;
    }
    else if(ev ->key()== Qt::Key_Left)
    {
        isLeftKeyDown_ = true;
        if(isRightKeyDown_)
            rosAngular = 0;
        else
            rosAngular = PER_ANGLER;
    }
    else if(ev ->key()== Qt::Key_Right)
    {
        isRightKeyDown_ = true;
        if(isLeftKeyDown_)
            rosAngular = 0;
        else
            rosAngular = -PER_ANGLER;
    }
    emit sigCarRun(rosLinear,rosAngular);
}

void MainWindow::keyReleaseEvent(QKeyEvent *ev)
{
     float rosLinear,rosAngular;
    if(ev ->key()== Qt::Key_Up)
    {
        isUpKeyDown_ = false;
        if(isDownKeyDown_)
            rosLinear = -PER_LINER;
        else
            rosLinear = 0;
    }
    else if(ev ->key()== Qt::Key_Down)
    {
        isDownKeyDown_ = false;
          if(isUpKeyDown_)
            rosLinear =PER_LINER;
        else
            rosLinear = 0;
    }
    else if(ev ->key()== Qt::Key_Left)
    {
        isLeftKeyDown_ = false;
        if(isRightKeyDown_)
            rosAngular =  -PER_ANGLER;
        else
            rosAngular = 0;
    }
    else if(ev ->key()== Qt::Key_Right)
    {
        isRightKeyDown_ = false;
           if(isLeftKeyDown_)
            rosAngular =  PER_ANGLER;
        else
            rosAngular = 0;
    }
    emit sigCarRun(rosLinear,rosAngular);
}