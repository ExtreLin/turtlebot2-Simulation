#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/types.h>
#include <unistd.h>
#include<stdlib.h>
#include<cstdlib>
#include<Eigen/Eigen>


MainWindow::MainWindow(int argc, char** argv, VulkanWindow* vulkanWindow,QWidget *parent) :
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
    QWidget *wrapper = QWidget::createWindowContainer(vulkanWindow);
    ui_->scrollArea->setWidget(wrapper);
    connect(ui_->testBtn, SIGNAL(clicked()),this,SLOT(slotTestBtnClicked()));
    qRegisterMetaType<sensor_msgs::CameraInfo>("sensor_msgs::CameraInfo"); 
    qRegisterMetaType<cv_bridge::CvImagePtr>("cv_bridge::CvImagePtr"); 
    qRegisterMetaType<Eigen::Matrix<float, 7,1>>("Eigen::Matrix<float, 7,1>");
    qRegisterMetaType<Eigen::Vector2f>("Eigen::Vector2f");
    qRegisterMetaType<TriMesh>("TriMesh");


    kLThread_ = new CKinectListenerThread(argc,argv);
    aThread_  = new  CAlgorithimThread();
    connect(kLThread_,SIGNAL(sigCvImage(const cv_bridge::CvImagePtr)),this,SLOT(slotCvImageRGB(const cv_bridge::CvImagePtr)));
    connect(kLThread_,SIGNAL(sigCvImageDepth(const cv_bridge::CvImagePtr)),this,SLOT(slotCvImageDepth(const cv_bridge::CvImagePtr)));
    connect(kLThread_,SIGNAL(sigCameraInfo(const sensor_msgs::CameraInfo&)),aThread_,SLOT(slotCameraInfo(const sensor_msgs::CameraInfo&)));
    connect(kLThread_,SIGNAL(sigCvImage(const cv_bridge::CvImagePtr)),aThread_,SLOT(slotCvImageRGB(const cv_bridge::CvImagePtr)));
    connect(kLThread_,SIGNAL(sigCvImageDepth(const cv_bridge::CvImagePtr)),aThread_,SLOT(slotCvImageDepth(const cv_bridge::CvImagePtr)));
    connect(kLThread_,SIGNAL(sigMoveFinished(const Eigen::Vector2f&)),aThread_,SLOT(slotMoveFinished(const Eigen::Vector2f&)));
    connect(this, SIGNAL(sigCarRun(const float&,const float&)),kLThread_,SLOT(slotCarRun(const float&,const float&)));
    connect(this, SIGNAL(sigInitScan()), aThread_,SLOT(slotInitScan()));
    connect(aThread_,SIGNAL(sigFinishInitScan()),this, SLOT(slotFinishInitScan()));
    connect(aThread_, SIGNAL(sigInitScanMoving()),this, SLOT(slotInitScanMoving()));
    connect(aThread_, SIGNAL(sigAutoScan()),aThread_, SLOT(slotAutoScan()));
    connect(aThread_, SIGNAL(sigGoNext()),aThread_, SLOT(slotGoNext()));
    connect(aThread_,SIGNAL(sigNavigation(const Eigen::Matrix<float,7,1>& )),kLThread_,SLOT(slotNavigation(const Eigen::Matrix<float,7,1>& )));
    
    connect(aThread_,&CAlgorithimThread::sigSendMesh,vulkanWindow,[=](const TriMesh& mesh){
        vulkanWindow->slotGetUpateMesh(mesh);
    });

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
    cv::Mat dist(cv_ptr->image.rows, cv_ptr->image.cols,CV_8U);

#pragma omp parallel for schedule(dynamic)
    for(int i=0;i<cv_ptr->image.rows;++i)
    {
        for(int j=0;j<cv_ptr->image.cols;++j)
        {
            dist.at<unsigned char>(i,j) = cv_ptr->image.at<float>(i,j) /5000 * 256;
        }
    }
    //cv_ptr->image.convertTo(dist,CV_8U);
    QImage qImg = QImage((const unsigned char*)(dist.data), dist.cols, dist.rows, dist.cols*dist.channels(), QImage::Format_Indexed8);
	qImg.setColorTable(colorTable_);//把qImg的颜色按像素点的颜色给设置
    ui_->depthImageLabel->setPixmap(QPixmap::fromImage(qImg));
    mutex2_.unlock();
}

void MainWindow::slotFinishInitScan()
{
    emit sigCarRun(0,0);
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

void MainWindow::slotTestBtnClicked()
{
    emit sigInitScan();
    slotInitScanMoving();
}

void MainWindow::slotInitScanMoving()
{
     emit sigCarRun(0,-PER_ANGLER);
}
