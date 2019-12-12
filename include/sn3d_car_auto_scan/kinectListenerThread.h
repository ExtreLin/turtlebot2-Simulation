#ifndef KINECT_LISTENER_THREAD_H
#define KINECT_LISTENER_THREAD_H

#include<QThread>
#include<QTimer>
#include"kinectListener.h"
#include <tf/transform_datatypes.h>
#include<Eigen/Eigen>

class CKinectListenerThread : public QThread{
    Q_OBJECT
public:
    CKinectListenerThread(int argc, char** argv):
    argc_(argc),
    argv_(argv),
    kl_(nullptr),
    isInit_(false),
    waitNum_(0)
    {
        qRegisterMetaType<sensor_msgs::CameraInfo>("sensor_msgs::CameraInfo"); 
        qRegisterMetaType<cv_bridge::CvImagePtr>("cv_bridge::CvImagePtr"); 
        qRegisterMetaType<Eigen::Matrix<float, 7,1>>("Eigen::Matrix<float, 7,1>");
        connect(&timer_,SIGNAL(timeout()),this, SLOT(slotTimeOut()));
        connect(this, SIGNAL(sigWaitMovingFinished()),this, SLOT(slotWaitMovingFinished()));
    };
    ~CKinectListenerThread()
    {
         ros::shutdown();
         if(kl_!=nullptr)
            delete kl_;
    }
    const tf::StampedTransform& get_current_rt();
protected:
    virtual void run();
private:
    int argc_;
    char** argv_;
    CKinectListener* kl_;
    bool isInit_;
    int  waitNum_;
    tf::StampedTransform stamped_transform_;   //定义存放变换关系的变量
    tf::StampedTransform original_rt;
    QTimer timer_;
public slots:
    void slotCvImage(const cv_bridge::CvImagePtr );
    void slotCvImageDepth(const cv_bridge::CvImagePtr );
    void slotCarRun(const float&,const float&);
    void slotCameraInfo(const sensor_msgs::CameraInfo&);
    void slotNavigation(const Eigen::Matrix<float,7,1>& pose);
    void slotWaitMovingFinished();
    void slotTimeOut();
signals:
    void sigCvImage(const cv_bridge::CvImagePtr );
    void sigCvImageDepth(const cv_bridge::CvImagePtr );
    void sigCameraInfo(const sensor_msgs::CameraInfo&);
    void sigWaitMovingFinished();
    void sigMoveFinished(const Eigen::Vector2f&);
};
#endif


