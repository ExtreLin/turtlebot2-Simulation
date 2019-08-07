#ifndef ALGORITHM_H
#define ALGORITHM_H
#include<sensor_msgs/CameraInfo.h>
#include<qobject.h>
#include<kinectfusion.h>

class Sn3DAlgorithmRebuild:public QObject
{
      Q_OBJECT
public:
    Sn3DAlgorithmRebuild():cameraInfo_(sensor_msgs::CameraInfo()),
    pipeline_(nullptr){
       configuration_.voxel_scale = 2.f;
       configuration_.init_depth = 700.f;
       configuration_.distance_threshold = 10.f;
       configuration_.angle_threshold = 20.f;
    }
    ~Sn3DAlgorithmRebuild(){
        if(pipeline_ !=nullptr)
            delete pipeline_;
    }
    void setCameraInfo(const   sensor_msgs::CameraInfo& cameraInfo);
private slots:

private:
    sensor_msgs::CameraInfo  cameraInfo_;
    kinectfusion::GlobalConfiguration configuration_;
    kinectfusion::Pipeline* pipeline_;
};
#endif