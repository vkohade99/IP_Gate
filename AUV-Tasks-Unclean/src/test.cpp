#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;
    int cannyLowGate = 150, cannyHighGate = 4, thresholdGate = 50, minLineLengthGate = 50, maxLineGapGate = 10;
    int cannyLowYaw = 10, cannyHighYaw = 65, thresholdYaw = 50, minLineLengthYaw = 50, maxLineGapYaw = 10;
    int N = 3;
    int N2 = 50;
    vector<vector<Vec4i> > lastNFrameLines;
    vector<float> lastAreas;
    float meanArea = 0.0;
	int morph_elem = 1;
int dilation_size = 0;
int morph_size = 1;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;
    int curIndex = -1;
    int areaIndex = -1;
	Point gateCenter;
void gate_detect(Mat);
void Morphology_Operations(Mat, Mat);

int main(int argc, char **argv){
	VideoCapture cap(argv[1]);
	if(!cap.isOpened()) return -1;
	namedWindow("Trackbars");

	createTrackbar( "Canny high gate", "Trackbars", &cannyHighGate, 255, NULL);

	Mat src, temp;
	Mat channel[3];
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  /// Generate grad_x and grad_y
  Mat grad_x, grad_y,grad,src_gray;
  Mat abs_grad_x, abs_grad_y;
		while(cap.isOpened()){
      Mat element = getStructuringElement( 0,
                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                         Point( dilation_size, dilation_size ) );
       Mat element1 = getStructuringElement( 0,
                          Size( 2*morph_size + 1, 2*morph_size+1 ),
                          Point( morph_size, morph_size ) );

		cap >> src;
		if(waitKey(10) == 'p')
			while(waitKey(10) == -1);
		if(src.empty())
			break;
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
  	HoughLinesP(grad, lines, 1, CV_PI/180, 150, 150, 30 );
    cout<<lines.size()<<endl;
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
               line( src, Point(lines[a][0], lines[a][1]), Point(lines[a][2], lines[a][3]), Scalar(0,0,255), 4, CV_AA);
             }
             if(b==-1 && abs(lines[a][0]-l[0])>100 && abs(abs(lines[i][1]-lines[i][3])-abs(l[1]-l[3]))<50)
             {
               b=i;
               line( src, Point(lines[b][0], lines[b][1]), Point(lines[b][2], lines[b][3]), Scalar(0,0,255), 4, CV_AA);
             }
        }
  	}
	imshow("orig",src);
	imshow( "g2", grad );
//morphologyEx( abs_grad_x, temp, 3, element );


		//imshow("Blue Removed", frame);
		//gate_detect(frame);
	}


return 0;
}
