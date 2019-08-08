#include<getRosData.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include<geometry_msgs/Twist.h>

  CKinectListener::CKinectListener():it_(nh_){
      sub_ = it_.subscribe("/camera/rgb/image_raw", 1000 ,&CKinectListener::imageCb,this);
      depthSub_ = it_.subscribe("/camera/depth/image_raw", 1000 ,&CKinectListener::depthCb,this);  
      //ptSub = nh.subscribe("/camera/depth/points",1000,&CKinectListener::pointsCb,this);
      camSub_  = nh_.subscribe("/camera/depth/camera_info",1000,&CKinectListener::cameraCb,this);
      runPub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
   } 

  void CKinectListener::imageCb(const sensor_msgs::ImageConstPtr& msg)
   {
      cv_bridge::CvImagePtr cv_ptr;   
      cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);   
      emit sigCvImage(cv_ptr);
   }

   void CKinectListener::depthCb(const sensor_msgs::ImageConstPtr& msg)
   {   
      cv_bridge::CvImagePtr cv_ptr;    
      cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);   
      emit sigCvImageDepth(cv_ptr);
   }

void CKinectListener::pointsCb(const sensor_msgs::PointCloud2& msg)
{
   pcl::PCLPointCloud2 pcl_pc;
   pcl_conversions::toPCL(msg, pcl_pc);    
   pcl::PointCloud<pcl::PointXYZ> input_cloud;
   pcl::fromPCLPointCloud2(pcl_pc, input_cloud);
   int sz = input_cloud.size();
}

void CKinectListener::cameraCb(const sensor_msgs::CameraInfo& msg)
{
   emit sigCameraInfo(msg);
    camSub_.shutdown();
}
