#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "getRosData.h"
#include<geometry_msgs/Twist.h>
#include <sys/types.h>
#include <unistd.h>

void CKinectListenerThread::slotCvImage(const cv_bridge::CvImagePtr qImage)
{
    emit sigCvImage(qImage);
}


void CKinectListenerThread::slotCvImageDepth(const cv_bridge::CvImagePtr qImage)
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
    if(isnan(liner)||isnan(angler))
        return;
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
    connect(kl_,SIGNAL(sigCvImage(const cv_bridge::CvImagePtr)),this,SLOT(slotCvImage(const cv_bridge::CvImagePtr)));
    connect(kl_,SIGNAL(sigCvImageDepth(const cv_bridge::CvImagePtr )),this,SLOT(slotCvImageDepth(const cv_bridge::CvImagePtr)));
    connect(kl_,SIGNAL(sigCameraInfo(const sensor_msgs::CameraInfo&)),this,SLOT(slotCameraInfo(const sensor_msgs::CameraInfo&)));
    spinner.start();
    ros::waitForShutdown();       
}


void CAlgorithimThread::run()
{
    QThread::exec();
}

void CAlgorithimThread::slotCameraInfo(const sensor_msgs::CameraInfo& msg)
{
    sn3dRebuild.setCameraInfo(msg);
}

void CAlgorithimThread::slotCvImageDepth(const cv_bridge::CvImagePtr& msg)
{
    sn3dRebuild.setCvImageDepth(msg);
}

void CAlgorithimThread::slotCvImageRGB(const cv_bridge::CvImagePtr& msg)
{
    sn3dRebuild.setCvImageRGB(msg);
}

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::MainWindow()),
    kLThread_(nullptr),
    aThread_(nullptr),
    mutex1_(),
    mutex2_(),
    isUpKeyDown_(false),
    isDownKeyDown_(false),
    isLeftKeyDown_(false),
    isRightKeyDown_(false)
{
    ui_->setupUi(this);
    qRegisterMetaType<sensor_msgs::CameraInfo>("sensor_msgs::CameraInfo"); 
    qRegisterMetaType<cv_bridge::CvImagePtr>("cv_bridge::CvImagePtr"); 
    kLThread_ = new CKinectListenerThread(argc,argv);
    aThread_  = new  CAlgorithimThread();
    connect(kLThread_,SIGNAL(sigCvImage(const cv_bridge::CvImagePtr)),this,SLOT(slotCvImageRGB(const cv_bridge::CvImagePtr)));
    connect(kLThread_,SIGNAL(sigCvImageDepth(const cv_bridge::CvImagePtr)),this,SLOT(slotCvImageDepth(const cv_bridge::CvImagePtr)));
    connect(kLThread_,SIGNAL(sigCameraInfo(const sensor_msgs::CameraInfo&)),aThread_,SLOT(slotCameraInfo(const sensor_msgs::CameraInfo&)));
    connect(kLThread_,SIGNAL(sigCvImage(const cv_bridge::CvImagePtr)),aThread_,SLOT(slotCvImageRGB(const cv_bridge::CvImagePtr)));
    connect(kLThread_,SIGNAL(sigCvImageDepth(const cv_bridge::CvImagePtr)),aThread_,SLOT(slotCvImageDepth(const cv_bridge::CvImagePtr)));
    connect(this, SIGNAL(sigCarRun(const float&,const float&)),kLThread_,SLOT(slotCarRun(const float&,const float&)));
    kLThread_->start();
    aThread_->start();
    this->grabKeyboard();
     for (int k = 0; k<256; ++k)
	{
		colorTable_.push_back(qRgb(k, k, k));
	}  
}

MainWindow::~MainWindow()
{
    kLThread_->terminate();
    delete ui_;
    delete kLThread_;
    delete aThread_;
}

void MainWindow::slotCvImageRGB(const cv_bridge::CvImagePtr cv_ptr)
{
   mutex1_.lock();
    QImage Img = QImage((const uchar*)(cv_ptr->image.data), cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.cols * cv_ptr->image.channels(), QImage::Format_RGB888);
    ui_->rgbImageLabel->setPixmap(QPixmap::fromImage(Img));
    mutex1_.unlock();
}

void MainWindow::slotCvImageDepth(const cv_bridge::CvImagePtr cv_ptr)
{
    mutex2_.lock();
    cv::Mat dist;
    cv_ptr->image.convertTo(dist,CV_8U);
    QImage qImg = QImage((const unsigned char*)(dist.data), dist.cols, dist.rows, dist.cols*dist.channels(), QImage::Format_Indexed8);
	qImg.setColorTable(colorTable_);//把qImg的颜色按像素点的颜色给设置
    ui_->depthImageLabel->setPixmap(QPixmap::fromImage(qImg));
    mutex2_.unlock();
}


void MainWindow::keyPressEvent(QKeyEvent *ev)
{
    float rosLinear,rosAngular;
    rosLinear = rosAngular = 0;
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