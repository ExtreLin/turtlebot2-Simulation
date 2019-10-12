#ifndef KINECT_LISTENER_THREAD_H
#define KINECT_LISTENER_THREAD_H

#include<QThread>
#include"kinectListener.h"
#include <tf/transform_datatypes.h>

class CKinectListenerThread : public QThread{
    Q_OBJECT
public:
    CKinectListenerThread(int argc, char** argv):
    argc_(argc),
    argv_(argv),
    kl_(nullptr)
    {
        qRegisterMetaType<sensor_msgs::CameraInfo>("sensor_msgs::CameraInfo"); 
        qRegisterMetaType<cv_bridge::CvImagePtr>("cv_bridge::CvImagePtr"); 
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
    tf::StampedTransform stamped_transform_;   //定义存放变换关系的变量
public slots:
    void slotCvImage(const cv_bridge::CvImagePtr );
    void slotCvImageDepth(const cv_bridge::CvImagePtr );
    void slotCarRun(const float&,const float&);
    void slotCameraInfo(const sensor_msgs::CameraInfo&);
signals:
    void sigCvImage(const cv_bridge::CvImagePtr );
    void sigCvImageDepth(const cv_bridge::CvImagePtr );
    void sigCameraInfo(const sensor_msgs::CameraInfo&);
};
#endif


