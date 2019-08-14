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

    int count;
    //取得支持Cuda的装置的数目
    cudaGetDeviceCount(&count);
    //没有符合的硬件
    if (count == 0) {
        fprintf(stderr, "There is no device.\n");
    }

    int i;

    for (i = 0; i < count; i++) {
        cudaDeviceProp prop;
        if (cudaGetDeviceProperties(&prop, i) == cudaSuccess) {
            if (prop.major >= 1) {
                break;
            }
        }
    }

    if (i == count) {
        fprintf(stderr, "There is no device supporting CUDA 1.x.\n");
    }
    cudaSetDevice(i);
    pipeline_  =  new  kinectfusion::Pipeline(camera_parameters,configuration_);
    scanNum = 0;
}

void Sn3DAlgorithmRebuild::getMesh()
{
    mutex1_.lock();
    pipeline_->process_frame(imgDepth_->image,imgRGB_->image);
    scanNum ++ ;
    if(scanNum >1000)
    {
        auto mesh = pipeline_->extract_mesh();
        for(int i=0;i<mesh.num_vertices;++i)
        {
            uchar3 &color = mesh.colors.at<uchar3>(i);
            std::swap(color.x,color.z);
        }
       kinectfusion::export_ply("mesh.ply", mesh);
       exit(1);
    }
    mutex1_.unlock();
}


void Sn3DAlgorithmRebuild::setCvImageRGB(const cv_bridge::CvImagePtr& imgRGB)
{
    imgRGB_ = imgRGB;
}



void Sn3DAlgorithmRebuild::setCvImageDepth(const cv_bridge::CvImagePtr& imgDepth)
{
    imgDepth_ = imgDepth;
    if(imgRGB_.get())
        getMesh();
}


