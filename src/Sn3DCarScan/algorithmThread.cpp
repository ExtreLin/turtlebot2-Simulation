#include "algorithmThread.h"

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