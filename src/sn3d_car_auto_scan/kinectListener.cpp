#include"kinectListener.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include<geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/PoseStamped.h>


  CKinectListener::CKinectListener():it_(nh_){
       sub_ = it_.subscribe("/camera/rgb/image_raw", 1000 ,&CKinectListener::imageCb,this);
      depthSub_ = it_.subscribe("/camera/depth/image_raw", 1000 ,&CKinectListener::depthCb,this);  
      //ptSub_ = nh_.subscribe("/camera/depth/points",1000,&CKinectListener::pointsCb,this);
      camSub_  = nh_.subscribe("/camera/depth/camera_info",1000,&CKinectListener::cameraCb,this);
      runSub_ = nh_.subscribe("/mobile_base/commands/velocity",1000,&CKinectListener::velocityCb,this);

      runPub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);
      goalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);
      //tfPub_ = nh_.advertise<geometry_msgs::PoseStamped>()
      //
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
#pragma omp parallel for schedule(dynamic)
      for(int i=0;i<cv_ptr->image.rows;++i)
      {
         for(int j=0;j<cv_ptr->image.cols;++j)
         {
            if(isnan(cv_ptr->image.at<float>(i,j)))
                cv_ptr->image.at<float>(i,j) = 0;
            else
                cv_ptr->image.at<float>(i,j) = cv_ptr->image.at<float>(i,j) *1000;
         }
      }
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

void CKinectListener::velocityCb(const geometry_msgs::Twist& cmd_vel)
{
   cmd_vel_ = cmd_vel;
}
