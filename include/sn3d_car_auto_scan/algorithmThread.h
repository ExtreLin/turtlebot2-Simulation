#ifndef ALGORITHIM_THREAD_H
#define ALGORITHIM_THREAD_H

#include<QThread>
#include"algorithm.h"


class CAlgorithimThread : public QThread{
    Q_OBJECT
public:
    CAlgorithimThread(){
        curr_rt_.setIdentity();
        initScanNum_ = 0;
        moveToThread(this);
        qRegisterMetaType<Eigen::Matrix<float, 7,1>>("Eigen::Matrix<float, 7,1>");
    }
public slots:  
    void slotCameraInfo(const sensor_msgs::CameraInfo&);
    void slotCvImageRGB(const cv_bridge::CvImagePtr&);
    void slotCvImageDepth(const cv_bridge::CvImagePtr&);
    void slotInitScan();
    void slotAutoScan();
    void slotGoNext();
    void slotMoveFinished(const Eigen::Vector2f&);
protected:
    virtual void run();
public:
    Sn3DAlgorithmRebuild sn3dRebuild_;
    Eigen::Matrix4f   curr_rt_;
    Eigen::Vector2f   car_curr_t;
    std::vector<Eigen::Matrix<float,7,1>>  scan_paths_;
    int curr_path_num_;
private:
    int initScanNum_;
signals:
    void sigAutoScan();
    void sigGoNext();
    void sigFinishInitScan();
    void sigInitScanMoving();
    void sigNavigation(const Eigen::Matrix<float,7,1>& pose);
    void sigSendMesh(const TriMesh& mesh);
};
#endif