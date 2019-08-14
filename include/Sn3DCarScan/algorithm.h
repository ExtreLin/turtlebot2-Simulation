#ifndef ALGORITHM_H
#define ALGORITHM_H
#include<qobject.h>
#include<QMutex>
#include<kinectfusion.h>
#include<cv_bridge.h>
#include<sensor_msgs/CameraInfo.h>

class Sn3DAlgorithmRebuild:public QObject
{
      Q_OBJECT
public:
    Sn3DAlgorithmRebuild():
    pipeline_(nullptr)
    {
       configuration_.voxel_scale = 10.f;
       configuration_.init_depth = 700.f;
       configuration_.distance_threshold =10.f;
       configuration_.angle_threshold = 5.f;
    }
    ~Sn3DAlgorithmRebuild(){
        if(pipeline_ !=nullptr)
            delete pipeline_;
    }
    void setCameraInfo(const   sensor_msgs::CameraInfo& cameraInfo);
    void setCvImageRGB(const cv_bridge::CvImagePtr&);
    void setCvImageDepth(const cv_bridge::CvImagePtr&);
    bool getRGBptr(){return imgRGB_.get();}
    void getMesh();
private:
    kinectfusion::GlobalConfiguration configuration_;
    kinectfusion::Pipeline* pipeline_;
    cv_bridge::CvImagePtr imgDepth_;
    cv_bridge::CvImagePtr imgRGB_;
    QMutex mutex1_;
    int  scanNum ;
};
#endif