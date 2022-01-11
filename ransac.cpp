#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
static const std::string OPENCV_WINDOW = "Image window";
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish to output video feed
    image_sub_ = it_.subscribe("/sensors/camera/infra1/image_rect_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

sensor_msgs::ImageConstPtr copy = msg;

cv_bridge::CvImagePtr cv_ptr,copy_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
   
    try
    {
      copy_ptr = cv_bridge::toCvCopy(copy, copy->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


     threshold(cv_ptr->image, cv_ptr->image,230, 255,cv::THRESH_BINARY);
     cv::circle(cv_ptr->image, cv::Point(340, 300), 35, CV_RGB(0,0,0),-1, 8,0);






//********************************************************************Beginning the RANSAC Procedure
//
float bestm;
int x1 = 500,y1=475,x2=11,y2=0,bestb,alsoinliers=0;//******************Draw two points uniformly at random
unsigned char* raw = cv_ptr->image.ptr<unsigned char>(y1);

while(raw[x1]<249)++x1;//************************ fixing the first point on the right most line
if(x1==750) return;
x1+=25;
cv::Point p1(x1, y1);//first point is now fixed
//
//
//
//***************************************Repeate N times to find the bestfit by adjusting the second point
for(int N=0;N<x1-50 ;++N)
	{ 
	   int currb;
	   float currm;
//****************************************fitting the the geometric line by calculating current m and b in the equation y=mx+b
currm= ((float)(y1-y2))/((float)(x1-x2));
currb=y1- currm*x1;
//***************************************************************
//
//
//
//***************************Search for possible inliers within the current Line
unsigned int currinliers=0,currx,curry=y1;
	for(;curry>y2;--curry)
{
currx=((float)(curry-currb))/currm -10 + 0.5;
//printf("currx=%i  %i  %i  %i  %i  %i\n",currx,N,x1,x2,y1,y2);
raw = cv_ptr->image.ptr<unsigned char>(curry);
for(int i=0;i<20;++i)if(raw[currx+i]>249) ++currinliers;	
}
//******************************************************************************
//
//
//******************************************************************compare and obtain bestm and bestb
if(alsoinliers<currinliers)
{
   alsoinliers=currinliers;
   bestm=currm;
   bestb=currb;
}
//****************************************************************************************************
//
//
//***************************************************************obtaining a possible better x2 and looping back
++x2;
	}//***********************End of N-loop
x2=(y2-bestb)/bestm;
printf("m1=%f  b1=%i\n",bestm,bestb);
cv::Point p2(x2, y2);
line(copy_ptr->image, p1, p2, cv::Scalar(0, 0, 0),10, cv::LINE_8);
//*******************************************************************************End of first  RANSAC Procedure








//********************************************************************Beginning the second RANSAC Procedure
//
float bestm2;
int x12 = 30,y12=475,x22=100,y22=0,bestb2,alsoinliers2=0;//******************Draw two points uniformly at random
unsigned char* raw2 = cv_ptr->image.ptr<unsigned char>(y12);

while(1){
while(raw2[x12]<249 && x12<300)++x12;//*************************** fixing the first point on the middle line 
raw2 = cv_ptr->image.ptr<unsigned char>(y12);
if(x12>=300){x12=30;y12-=5;}else break;
}

x12+=25;
cv::Point p12(x12, y12);//first point is now fixed
//
//
//
//***************************************Repeate N times to find the bestfit by adjusting the second point
for(int N=x22;N<x12 +200 ;++N)
        {
		if(x22==x12){++x22;continue;}
           int currb;
           float currm;
//****************************************fitting the the geometric line by calculating current m and b in the equation y=mx+b
currm= ((float)(y12-y22))/((float)(x12-x22));
currb=y12-currm*x12;
//***************************************************************
//
//
//
//***************************Search for possible inliers within the current Line
int currinliers=0,currx,curry=y12;
        for(;curry>y22;--curry)
{
currx=((float)(curry-currb))/currm -10 + 0.5;
raw2 = cv_ptr->image.ptr<unsigned char>(curry);
for(int i=0;i<20;++i)if(raw2[currx+i]>249) ++currinliers;
}
//******************************************************************************
//
//
//******************************************************************compare and obtain bestm2 and bestb2
if(alsoinliers2<currinliers)
{
   alsoinliers2=currinliers;
   bestm2=currm;
   bestb2=currb;
}
//****************************************************************************************************
//
//
//***************************************************************obtaining a possible better x2 and looping back
++x22;
        }//***********************End of N-loop
x22=(y22-bestb2)/bestm2;

printf("m2=%f  b2=%i\n",bestm2,bestb2);
cv::Point p22(x22, y22);
line(copy_ptr->image, p12, p22, cv::Scalar(0, 0, 0),10, cv::LINE_8);
//*******************************************************************************End of RANSAC Procedure












/*

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
 */



   // Update GUI Window
   //
   //
    cv::imshow(OPENCV_WINDOW, copy_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(copy_ptr->toImageMsg());
  }
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;

  ImageConverter ic;//************************************* image subscriber and publisher



  ros::spin();

  return 0;

}



















