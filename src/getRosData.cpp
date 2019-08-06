#include<getRosData.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include<geometry_msgs/Twist.h>

  CKinectListener::CKinectListener():it(nh){
      sub = it.subscribe("/camera/rgb/image_raw", 1000 ,&CKinectListener::imageCb,this);
      depthSub = it.subscribe("/camera/depth/image_raw", 1000 ,&CKinectListener::depthCb,this);  
      //ptSub = nh.subscribe("/camera/depth/points",1000,&CKinectListener::pointsCb,this);
      runPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
      for (int k = 0; k<256; ++k)
		{
			colorTable.push_back(qRgb(k, k, k));
		}  
   } 

  void CKinectListener::imageCb(const sensor_msgs::ImageConstPtr& msg)
   {
      cv_bridge::CvImagePtr cv_ptr;   
      cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);   
      QImage Img = QImage((const uchar*)(cv_ptr->image.data), cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.cols * cv_ptr->image.channels(), QImage::Format_RGB888);
      emit sigCvImage(Img);
   }

   void CKinectListener::depthCb(const sensor_msgs::ImageConstPtr& msg)
   {   
      cv_bridge::CvImagePtr cv_ptr;    
      cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);   
      cv::Mat dist;
      cv_ptr->image.convertTo(dist,CV_8U);
      QImage qImg = QImage((const unsigned char*)(dist.data), dist.cols, dist.rows, dist.cols*dist.channels(), QImage::Format_Indexed8);
		qImg.setColorTable(colorTable);//把qImg的颜色按像素点的颜色给设置
      emit sigCvImageDepth(qImg);
}

void CKinectListener::pointsCb(const sensor_msgs::PointCloud2& msg)
{
   pcl::PCLPointCloud2 pcl_pc;
   pcl_conversions::toPCL(msg, pcl_pc);    
   pcl::PointCloud<pcl::PointXYZ> input_cloud;
   pcl::fromPCLPointCloud2(pcl_pc, input_cloud);
   int sz = input_cloud.size();
   for(int i=0;i<input_cloud.size();++i)
   {
      float x = input_cloud.at(i).x;
      float y = input_cloud.at(i).y;
      float z = input_cloud.at(i).z;
      if(isnan(x)||isnan(y)||isnan(z))
          continue;
   }
}
