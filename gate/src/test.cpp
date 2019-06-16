#include<ros/ros.h>
//#include<opencv2/opencv.hpp>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

	Mat src, temp;
  int cannyHighGate = 4;
int dilation_size = 0;
int morph_size = 1;
int scale = 1;
int delta = 0;
int ddepth = CV_16S;

///////
void call(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //imshow("view1",cv_bridge::toCvShare(msg,"bgr8")->image);
    src=cv_bridge::toCvShare(msg,"bgr8")->image;
    //if(waitKey(30)=='k') std::exit(0);
    Mat grad_x, grad_y,grad,src_gray;
    Mat abs_grad_x, abs_grad_y;

        Mat element = getStructuringElement( 0,
                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                           Point( dilation_size, dilation_size ) );
         Mat element1 = getStructuringElement( 0,
                            Size( 2*morph_size + 1, 2*morph_size+1 ),
                            Point( morph_size, morph_size ) );

  		//cap >> src;
  		if(waitKey(10) == 'p')
  			while(waitKey(10) == -1);
  	
      cvtColor( src, src_gray, CV_BGR2GRAY );
    GaussianBlur( src_gray, src_gray, Size( 5,5 ), 0, 0,BORDER_DEFAULT );

    Sobel( src_gray, grad_x, ddepth, 1, 0, 1, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    //Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    //convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_x, 0.5, 0, grad );
  	threshold(grad,grad,cannyHighGate,255, THRESH_BINARY);
    erode(grad,grad,element);
  	morphologyEx( grad, grad, 1, element1 );
    medianBlur( grad, grad, 5);

    morphologyEx( grad, grad, 2, element1 );
  	vector<Vec4i> lines;
    	HoughLinesP(grad, lines, 1, CV_PI/180, 110, 110, 35 );
      if(lines.size()==0)
      {
        addWeighted( abs_grad_x, 0.5, abs_grad_x, 0.5, 0, grad );
        threshold(grad,grad,2,255, THRESH_BINARY);
        erode(grad,grad,element);
      	morphologyEx( grad, grad, 1, element1 );
        medianBlur( grad, grad, 5);

        morphologyEx( grad, grad, 2, element1 );
        HoughLinesP(grad, lines, 1, CV_PI/180, 75, 75, 25 );
      }
      //cout<<lines.size()<<endl;
      int a=-1,b=-1;
    	for( size_t i = 0; i < lines.size(); i++ )
    	{
      	Vec4i l = lines[i];
  		double angle = atan2 (abs(l[1]-l[3]),abs(l[0]-l[2])) * 180 / M_PI;
          if(abs(90.0-angle) < 5)
          {
               if(a==-1)
               {
                 a=i;
               }
               if(b==-1 && abs(lines[a][0]-l[0])>100 && abs(abs(lines[a][1]-lines[a][3])-abs(l[1]-l[3]))<50)
               {
                 int p,q;
                 p=min(lines[a][1],lines[a][3]);
                 q=min(l[1],l[3]);
                 if(abs(p-q)<30)
                 {
                   b=i;
                   line( src, Point(lines[a][0], lines[a][1]), Point(lines[a][2], lines[a][3]), Scalar(0,0,255), 4, CV_AA);
                   line( src, Point(lines[b][0], lines[b][1]), Point(lines[b][2], lines[b][3]), Scalar(0,255,0), 4, CV_AA);
                 }
               }
               //else a=i;

               if(a!=-1 && b!=-1)
               {
                 int x_center = (lines[a][0]+lines[a][2]+lines[b][0]+lines[b][2])/4;
                 int y_center = (lines[a][1]+lines[a][3]+lines[b][1]+lines[b][3])/4;
                 circle(src,Point(x_center,y_center),1,Scalar(0,0,255),50,8,0);
                 //circle(grad,Point(x_center,y_center),1,Scalar(0,0,255),50,8,0);
                 break;
               }
          }
    	}
  	imshow("orig",src);
  	imshow( "g2", grad );

  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("ERROR");
  }
}



int main(int argc, char **argv){

  ros::init(argc,argv,"image_subscriber1");
  ros::NodeHandle nh;
  //cv::namedWindow("view1",CV_WINDOW_NORMAL);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub=it.subscribe("/front_camera/image_rect_color",1000,call);
  ros::spin();


return 0;
}
