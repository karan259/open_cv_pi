#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/video/tracking.hpp"
#include "RaspiCamCV.h"
using namespace std; 
using namespace cv;
 
int main ( int argc,char **argv ) {
	time_t timer_begin,timer_end;
	RaspiCamCvCapture* camera = raspiCamCvCreateCameraCapture(0);
    	Mat image;
    	int nCount= 0;
    //set camera params
    //Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    //Creating window
    	namedWindow( "Project@Dexter_Industry", 0 );
    	cout<<"Opening Camera..."<<endl;
    
	//Check  for streaming
	if ( !camera ) { 
		cerr<<"Error opening the camera"<< endl;return -1; 
	}
	
    //Start capture
    	//cout<< "Capturing "<< nCount <<" frames from Camera ...."<< endl;
    	time ( &timer_begin );
	
    	while( true ) {
        	image = raspiCamCvQueryFrame( camera );
		cvtColor( image, image, CV_BGR2GRAY );
        	GaussianBlur(image, image, Size( 9, 9 ), 2, 2 );
		threshold(image,image, 30, 255, 1);

		Moments mu;
		mu = moments(image, true);
		Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
		cout /*<< "% of x and y "*/<<(1-center.x/image.cols)*100<<"\t"<<(1-center.y/image.rows)*100;
		//cout << "x and y coord of center\t" << center.x << "\t" << center.y <<endl ;
		circle(image, center,5, Scalar(0,255,0),CV_FILLED, 8,0);
		
		//imshow("Project@Dexter_Industry",image);
        //if ( i%5==0 )  cout<<"\r captured "<<i<<" images"<<std::flush;
		
		if( waitKey(5) == 27)
			break;
			
		//cout<<"\r captured "<<nCount + 1<<" images"<<std::flush;
		nCount++;
    	}
	
    	//cout<<"Stop camera..."<<endl;
    	//raspiCamCvReleaseCapture( &camera );
	//destroyWindow("Project@Dexter_Industry");
	
    //show time statistics
    	time ( &timer_end ); /* get current time; same as: timer = time(NULL)  */
    	double secondsElapsed = difftime ( timer_end,timer_begin );
    	cout<< secondsElapsed<<" seconds for "<< nCount<<"  frames : FPS = "<<  ( float ) ( ( float ) ( nCount ) /secondsElapsed ) <<endl;
    
	//save image
    	imwrite("raspicam_cv_image.jpg",image);
	raspiCamCvReleaseCapture(&camera);
	destroyWindow("Project@Dexter_Industry");
    	cout<<"Final Image saved at raspicam_cv_image.jpg"<<endl;
}

