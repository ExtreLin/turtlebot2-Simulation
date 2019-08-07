#include "algorithm.h"

void Sn3DAlgorithmRebuild::setCameraInfo(const   sensor_msgs::CameraInfo& cameraInfo )
{
    cameraInfo_= cameraInfo;
    kinectfusion::CameraParameters  camera_parameters;
    camera_parameters.focal_x = cameraInfo.K[0];
    camera_parameters.focal_y = cameraInfo.K[4];
    camera_parameters.principal_x = cameraInfo.K[2];
    camera_parameters.principal_y = cameraInfo.K[5];
    camera_parameters.image_width = cameraInfo.width;
    camera_parameters.image_height = cameraInfo.height;
    pipeline_ = new  kinectfusion::Pipeline(camera_parameters,configuration_);
}



