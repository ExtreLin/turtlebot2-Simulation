#ifndef ALGORITHIM_THREAD_H
#define ALGORITHIM_THREAD_H

#include<QThread>
#include"algorithm.h"


class CAlgorithimThread : public QThread{
    Q_OBJECT
public:
    CAlgorithimThread(){
        moveToThread(this);
    }
public slots:  
    void slotCameraInfo(const sensor_msgs::CameraInfo&);
    void slotCvImageRGB(const cv_bridge::CvImagePtr&);
    void slotCvImageDepth(const cv_bridge::CvImagePtr&);
protected:
    virtual void run();
public:
        Sn3DAlgorithmRebuild sn3dRebuild;
};
#endif