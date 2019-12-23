#include "algorithm.h"
#include"qthread.h"

void Sn3DAlgorithmRebuild::setCameraInfo(const   sensor_msgs::CameraInfo& cameraInfo )
{
    kinectfusion::CameraParameters  camera_parameters;
    camera_parameters.focal_x = cameraInfo.K[0];
    camera_parameters.focal_y = cameraInfo.K[4];
    camera_parameters.principal_x = cameraInfo.K[2];
    camera_parameters.principal_y = cameraInfo.K[5];
    camera_parameters.image_width = cameraInfo.width;
    camera_parameters.image_height = cameraInfo.height;
    pipeline_  =  new  kinectfusion::Pipeline(camera_parameters,configuration_);
}

void Sn3DAlgorithmRebuild::processFrame()
{
    mutex1_.lock();
    pipeline_->process_frame(imgDepth_->image,imgRGB_->image);
    mutex1_.unlock();
}

void Sn3DAlgorithmRebuild::setCvImageRGB(const cv_bridge::CvImagePtr& imgRGB)
{
    imgRGB_ = imgRGB;
}

void Sn3DAlgorithmRebuild::setCvImageDepth(const cv_bridge::CvImagePtr& imgDepth)
{
    imgDepth_ = imgDepth;
}

void Sn3DAlgorithmRebuild::getMeshAutoScan(const Eigen::MatrixX4f& rt)
{
    mutex1_.lock();
    pipeline_->process_frame_by_rt(imgDepth_->image,imgRGB_->image,  rt);
    mutex1_.unlock();
}

TriMesh Sn3DAlgorithmRebuild::getMesh()
{
    kinectfusion::SurfaceMesh smesh =  pipeline_->extract_mesh();
    TriMesh tmesh;
    kinectfusion::surfacemesh_to_TriMesh(smesh, tmesh);
    return tmesh;
}

std::vector<Eigen::Matrix<float,7,1> > Sn3DAlgorithmRebuild::getScanPath(std::function<void(const TriMesh& )> func)
{
    return pipeline_->compute_paths(func);
}

void Sn3DAlgorithmRebuild::pose2RT(const Eigen::Matrix<float,7,1>& pose, Eigen::Matrix4f& rt)
{
    Eigen::Vector3f t = Eigen::Vector3f (pose[0],pose[1],pose[2]);
    Eigen::Quaternionf quaternion;
    quaternion.x() =pose[3];
    quaternion.y() =pose[4];
    quaternion.z() =pose[5];
    quaternion.w() =pose[6];
    Eigen::Matrix3f r =  quaternion.matrix();
    //这些RT是地图坐标系的，将其转换成move_base的坐标系下
    //t的转换 在算法中的世界坐标系是在体素块的原点而不是在体素块的中心点
    t.z() = t.x()*1000;
    t.x() = -t.y()*1000;
    t.y() = 0;
    //r的转换
    Eigen::Vector3f el =  r.eulerAngles(0, 1, 2);
    //
    Eigen::AngleAxisf  angleAxis(el.z(), Eigen::Vector3f(0, -1, 0));
    Eigen::Matrix3f tmpR  = angleAxis.matrix();

    rt.block(0, 0, 3, 3) = tmpR;
    Eigen::Vector3f new_t = t ;
    rt.block(0, 3, 3, 1) =new_t;
}






