#include "algorithmThread.h"


void CAlgorithimThread::run()
{
    TriMesh mesh;
    OpenMesh::IO::read_mesh(mesh,"tmp/meshout.ply");
    sigSendMesh(mesh);
    QThread::exec();
}

void CAlgorithimThread::slotCameraInfo(const sensor_msgs::CameraInfo& msg)
{
    sn3dRebuild_.setCameraInfo(msg);
}

void CAlgorithimThread::slotCvImageDepth(const cv_bridge::CvImagePtr& msg)
{
    if(sn3dRebuild_.getScanStatus() == ScanStatus::isInitialScan||
        sn3dRebuild_.getScanStatus() == ScanStatus::isAutoScan)
    {
        sn3dRebuild_.setCvImageDepth(msg);
        if(!sn3dRebuild_.getRGBptr())
            return;
        if(sn3dRebuild_.getScanStatus() == ScanStatus::isInitialScan)
        {
            sn3dRebuild_.getMesh();
            initScanNum_++;
            if(initScanNum_ == 200)
            {
                sn3dRebuild_.setScanStatus(ScanStatus::isWait);
                emit sigFinishInitScan();
                emit sigAutoScan();
            }  
            else
                emit sigInitScanMoving();      
        }

        if(sn3dRebuild_.getScanStatus() == ScanStatus::isAutoScan)
        {
             if(abs(car_curr_t.x()-scan_paths_[curr_path_num_-1][0])<0.5&&
                 abs(car_curr_t.y()-scan_paths_[curr_path_num_-1][1])<0.5)
                sn3dRebuild_.getMeshAutoScan(curr_rt_);

            sn3dRebuild_.setScanStatus(ScanStatus::isWait);
            emit sigGoNext();
        }    
    }
}

void CAlgorithimThread::slotCvImageRGB(const cv_bridge::CvImagePtr& msg)
{
    if(sn3dRebuild_.getScanStatus() == ScanStatus::isInitialScan||
        sn3dRebuild_.getScanStatus() == ScanStatus::isAutoScan)
        sn3dRebuild_.setCvImageRGB(msg);
}

void CAlgorithimThread::slotInitScan()
{
    sn3dRebuild_.setScanStatus(ScanStatus::isInitialScan);
}

void CAlgorithimThread::slotAutoScan()
{
    scan_paths_ =  sn3dRebuild_.getScanPath();
    curr_path_num_ =0;
    emit sigGoNext();
}

void CAlgorithimThread::slotGoNext()
{
    if( curr_path_num_  == scan_paths_.size())
        emit sigAutoScan();
    else
    {
        sn3dRebuild_.pose2RT(scan_paths_[curr_path_num_],curr_rt_);
        //发送信号小车走动
        emit sigNavigation(scan_paths_[curr_path_num_]);
    }
}

void CAlgorithimThread::slotMoveFinished(const Eigen::Vector2f& curr_t)
{
    car_curr_t = curr_t;
    curr_path_num_++;  
    sn3dRebuild_.setScanStatus(ScanStatus::isAutoScan);
}


