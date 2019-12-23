#ifndef ALGORITHM_H
#define ALGORITHM_H
#include<qobject.h>
#include<QMutex>
#include<kinectfusion.h>
#include<cv_bridge.h>
#include<sensor_msgs/CameraInfo.h>

enum class ScanStatus
{
    isWait,
    isInitialScan,
    isAutoScan
};

class Sn3DAlgorithmRebuild:public QObject
{
      Q_OBJECT
public:
    Sn3DAlgorithmRebuild():
    pipeline_(nullptr),
    status_(ScanStatus::isWait)
    {
       configuration_.voxel_scale = 14.f;
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
    void processFrame();
    void getMeshAutoScan(const Eigen::MatrixX4f& rt);
    void pose2RT(const Eigen::Matrix<float,7,1>& pose,Eigen::Matrix4f& rt);
    TriMesh getMesh();
   std::vector<Eigen::Matrix<float,7,1> > getScanPath(std::function<void(const TriMesh& )> );
    ScanStatus getScanStatus(){return status_;}
    ScanStatus setScanStatus(const ScanStatus& ss ) {status_ = ss;}
private:
    kinectfusion::GlobalConfiguration configuration_;
    kinectfusion::Pipeline* pipeline_;
    cv_bridge::CvImagePtr imgDepth_;
    cv_bridge::CvImagePtr imgRGB_;
    QMutex mutex1_;
    ScanStatus status_;
};
#endif