#include "kinectListenerThread.h"
#include<geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const tf::StampedTransform& CKinectListenerThread::get_current_rt()
{
    return stamped_transform_;
}

void CKinectListenerThread::slotCvImage(const cv_bridge::CvImagePtr qImage)
{
    emit sigCvImage(qImage);
}


void CKinectListenerThread::slotCvImageDepth(const cv_bridge::CvImagePtr qImage)
{
    emit sigCvImageDepth(qImage);
}

void CKinectListenerThread::slotCarRun(const float& liner,const float& angler)
{
   if(kl_ == nullptr)
        return;
    geometry_msgs::Twist speed;
    speed = geometry_msgs::Twist();
    speed.linear.x = liner;
    speed.angular.z = angler;  
    if(isnan(liner)||isnan(angler))
       return;
    kl_->tflistener_ .lookupTransform("odom", "base_footprint",ros::Time(0), stamped_transform_);       
    kl_->runPub_.publish(speed);   
} 

void CKinectListenerThread::slotCameraInfo(const sensor_msgs::CameraInfo& msg)
{
    emit sigCameraInfo(msg);
}

void CKinectListenerThread::run(){
    ros::init(argc_,argv_,"my_Kinect_listener");
    kl_ = new  CKinectListener();
    ros::AsyncSpinner spinner(8); // Use8 threads
    connect(kl_,SIGNAL(sigCvImage(const cv_bridge::CvImagePtr)),this,SLOT(slotCvImage(const cv_bridge::CvImagePtr)));
    connect(kl_,SIGNAL(sigCvImageDepth(const cv_bridge::CvImagePtr )),this,SLOT(slotCvImageDepth(const cv_bridge::CvImagePtr)));
    connect(kl_,SIGNAL(sigCameraInfo(const sensor_msgs::CameraInfo&)),this,SLOT(slotCameraInfo(const sensor_msgs::CameraInfo&)));
    spinner.start();
    ros::waitForShutdown();       
}