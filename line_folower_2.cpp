/*
########################################################################                                                                 
# Program Name: Pi_Cam_Line_Follower.cpp                                     
# ================================     
# This code is for making an autonomous line follower bot.
# The bot can follow Black and White colored line.                                  
# http://www.dexterindustries.com/                                                                
# History
# ------------------------------------------------
# Author       Date      Comments
# Govinda      01/07/14  Initial Authoring
#                                                                  
# These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
# (http://creativecommons.org/licenses/by-sa/3.0/)           
#
########################################################################
*/
// Command:
// Press Esc from your keyboard - To stop the program

/* 
How to compile:

g++ `pkg-config --cflags opencv` my_code.cpp -o my_code `pkg-config --libs opencv` -I/home/pi/git/robidouille/raspicam_cv -L/home/pi/git/robidouille/raspicam_cv -lraspicamcv -L/home/pi/git/raspberrypi/userland/build/lib -lmmal_core -lmmal -l mmal_util -lvcos -lbcm_host -lrt -lm -L/usr/local/lib -lwiringPi

*/

/*
How to run:

./my_code thresold_value speed	is_black show_image ROI_y ROI_height

threshold_vlaue - is a threshold for detection. Keep it higher if you are trying to detect white otherwise keep it lower if you trying to detect black
						threshold ranges from 0-255.
speed - Normal speed with which bot will forward

is_black - 0 for detecting white and 1 for detecting black

show_image - An optional argument. By default it shows the image. If you don't want to see the image what it tracks then give it 0.

ROI_y - An optional argument to shift your ROI in y direction. By default it is at 1/3 of  y axix. Since y-axis is constrained by the no of rows of image matrix
			Enter this in percentage form like if you want to make the origin of ROI as 90% of the y-axis of the original image then enter as 90.

ROI_height - An optional argument to change the height of ROI. By default it is 1/6 of the original height. Enter this as explained above for ROI_y.

*/

#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/video/tracking.hpp"
#include "RaspiCamCV.h"

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "tick.h"
#include <wiringPi.h>
#include "BrickPi.h"
#include <linux/i2c-dev.h>  
#include <fcntl.h>

using namespace std; 
using namespace cv;

int result,speed_1,speed_2;
int motor1,motor2;
Moments mu;

#undef DEBUG

//Move Forward
void fwd()
{
	BrickPi.MotorSpeed[motor1] = speed_1;
	BrickPi.MotorSpeed[motor2] = speed_2;
}

//Move Left
void left()
{
	BrickPi.MotorSpeed[motor1] = speed_1;  
	BrickPi.MotorSpeed[motor2] = -speed_2;
}

//Move Right
void right()
{
	BrickPi.MotorSpeed[motor1] = -speed_1;  
	BrickPi.MotorSpeed[motor2] = speed_2;
}

//Move backward
void back()
{
	BrickPi.MotorSpeed[motor1] = -speed_1;  
	BrickPi.MotorSpeed[motor2] = -speed_2;
}

//Stop
void stop()
{
	BrickPi.MotorSpeed[motor1] = 0;  
	BrickPi.MotorSpeed[motor2] = 0;
}	

/*
The x_Offset function takes two arguments threshold and is_black and three optional arguments for finding region of interest.
ROI_y, ROI_width and ROI_height are the optional variables because ROI_x is 0 in this function.
If the user will not give these optional arguments then this function will set those variable as default. 
This function returns the offset from the center of the x coordinate of the center of the white pixels on either side.
The value of offset rnges from -1 to 1. This returns -2 if it could not find the center.
*/
float x_Offset(Mat imgROI, int width)
{	
	

	/*
	Moment is class to store some information about the pixel
	values, from which we can easily get the center of white
	pixels.
	Inside the Moments definition:
	m00 - total no. of white pixels
	m10 - sum of x coordinate where pixels are white
	m01 - sum of y coordinate where pixels are white
	moments(InputArray array, bool binaryImage) is a function 	which returns Moments.
	*/
	
	/*
	Here the argument is true because imgROI is binary image
	Black & white image is a binary images.
	*/
	mu = moments(imgROI, true); 
	
	//A simple logic to calculate x coordinate of the center.
	Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
	
	float offset = (1-2*center.x/width);

	// nan stands for not a number.When the floating point number cannot be represented by the system. 
	// Cases like 0/0 or some other indeterminate number for which the system cannot represent it. Then it assigns it to nan
	// isnan() is which gives a boolean value true if the number is nan otherwise false 
	if(isnan(offset))	
		offset = -2;		// To avoid printing nan

	//y_offset = (1-center.y/image.rows);
	cout<<offset<<"\n"<<flush;		// flush is here to flush the buffer.
	
	return offset;
}

void move_bot(float offset, int normal_speed)
{	
	if(mu.m00>2500)
	{
		speed_1 = speed_2 = normal_speed;	//Set the normal speed 
		
		if(fabs(offset)>0.4 && fabs(offset)<0.6)	// If the offset is more than 70% on either side from the center of the image
		{
			//if((normal_speed+normal_speed*0.05)<255)
				speed_1=speed_2= 50; //Set the speed to 50
			
			if(offset>0)		// If the x coordinate of the path is on the left side from the center
			{
				speed_2 = 0;		//Stop the left wheel	
				fwd();	//Run forward with left wheel stopped i.e turn left
				//left();	
				BrickPiUpdateValues();	//Update the motor values
				//cout<<"l_0.4"<<endl;
			}
			else
			{	
				speed_1 = 0;	//Stop the right wheel
				fwd();	//Run forward with right wheel stopped i.e turn right
				//right();
				BrickPiUpdateValues();	//Update the motor values
				//cout<<"r_0.4"<<endl;
			}	
		}
		else if(fabs(offset)>0.6 )//&& fabs(offset)<0.6)	// If the offset is more than 70% on either side from the center of the image
		{
			//if((normal_speed+normal_speed*0.1)<255)
				speed_1=speed_2= 60;	//Set the seed to 60
			//else	
				//speed_1=speed_2=255;
			if(offset>0)		// If the x coordinate of the path is on the left side from the center
			{
				speed_2 = 0;		//Stop the left wheel
				fwd();	//Run forward with left wheel stopped i.e turn left	
				//left();	
				BrickPiUpdateValues();	//Update the motor values
				//cout<<"l_0.4"<<endl;
			}
			else
			{	
				speed_1 = 0;		//Stop the right wheel
				fwd();	//Run forward with right wheel stopped i.e turn right
				//right();
				BrickPiUpdateValues();	//Update the motor values
				//cout<<"r_0.4"<<endl;
			
			}	
		}
		/*else if(fabs(offset)>0.6 && fabs(offset)<0.8)	// If the offset is more than 70% on either side from the center of the image
		{
			//if((normal_speed+normal_speed*0.15)<255)
				speed_1=speed_2=60;//normal_speed+normal_speed*0.15;		// Increase the speed of both wheels by 30%
			//else	
				//speed_1=speed_2=255;
			if(offset>0)		// If the x coordinate of the path is on the left side from the center
			{	
				left();	
				BrickPiUpdateValues();	//Update the motor values
				//cout<<"l_0.6"<<endl;
			}
			else
			{	
				right();
				BrickPiUpdateValues();	//Update the motor values
				//cout<<"r_0.6"<<endl;
			
			}	
		}
		else if(fabs(offset)>0.8)	// If the offset is more than 90% on either side from the center of the image
		{
			//if((normal_speed+normal_speed*0.2)<255)
				speed_1=speed_2=60;//normal_speed+normal_speed*0.2;		// Increase the speed of both wheels by 30%
			//else	
				//speed_1=speed_2=255;
			if(offset>0)		// If the x coordinate of the path is on the left side from the center
			{	
				left();	
				BrickPiUpdateValues();	//Update the motor values
				//cout<<"l_0.8"<<endl;
			}
			else
			{	
				right();
				BrickPiUpdateValues();	//Update the motor values
				//cout<<"r_0.8"<<endl;
			
			}	
		}*/
		else	// Move forward with the specified speed by the user
		{
			//speed_1=speed_2=normal_speed;		
			fwd();
			BrickPiUpdateValues();	//Update the motor values
			//cout<<"s"<<endl;
		}
	}
}

int main(int argc,char **argv) 
{
	ClearTick();

	result = BrickPiSetup();
	// printf("BrickPiSetup: %d\n", result);
	if(result)
		return 0;

	BrickPi.Address[0] = 1;
	BrickPi.Address[1] = 2;

	motor1=PORT_B;	//Select the ports to be used by the motors
	motor2=PORT_C; 

	BrickPi.MotorEnable[motor1] = 1;	//Enable the motors
	BrickPi.MotorEnable[motor2] = 1;

	result = BrickPiSetupSensors();		//Set up the properties of sensors for the BrickPi
	//printf("BrickPiSetupSensors: %d\n", result); 

	BrickPi.Timeout=1000;			//Set timeout value for the time till which to run the motors after the last command is pressed
	BrickPiSetTimeout();				//Set the timeout

	// OpenCV Part Starts
	
	Mat image;	// Variable to store the image taken from the camera
	Mat imgROI;	// To store the region of interest which is a subset of the image
	float offset;	// Offset from center
	
	int thresh = atoi(argv[1]);	//	threshold is the first parameter given by the user
	int normal_speed= atoi(argv[2]);	//	Speed is the second parameter given by the user
	int is_black = atoi(argv[3]);	// is_black is 0 for white and 1 for black. Third parameter given by the user
	int show_image =1;
	int ROI_y, ROI_width, ROI_height;	// Parameters for region of interest. These are the optional parameter provided by the user.
	bool not_assigned = true;	//Just to ensure that the region of interest is properly defined
	if(argc >=5)
		show_image = atoi(argv[4]);	// This is just an optional variable to disable the video streaming on screen.		

	RaspiCamCvCapture* camera = raspiCamCvCreateCameraCapture(0);	// Declare a variable to handle the streaming from Raspberry Pi Camera
    
	if( show_image )	// If the show_image is 1 then it opens the window otherwise it does not 
		namedWindow( "Project@Dexter_Industry", 0 );	// Give a name to window on which images will be getting shown.
		
    cout<<"Opening Camera..."<<endl;
	
	// A check  for streaming
	if ( !camera )
	{ 
		cerr<<"Error opening the camera"<< endl;return -1; 
	}
	
	if(!result)
	{
		while(true)		// An infinite loop
		{	
			image = raspiCamCvQueryFrame( camera );	// Get the frame from which is captured by camera
			
			//Initialize the ROI
			if(not_assigned)
			{
				if(argc == 7)	//If user specify the arguments of the ROI
				{
					ROI_y = image.rows*atoi(argv[5])/100.00;		//Percentage by which orgin of the ROI has to be shifted in y direction.
					ROI_width = image.cols;	//Width of the ROI
					ROI_height = image.rows*atoi(argv[6])/100.00;	//Height of the ROI in terms of the percentage of the original image.
				}
				else 
				{
					ROI_y = image.rows/3;	//Origin of the ROI is shifted by 1/3 of the y-axis of the original image.
					ROI_width = image.cols;	//Width of the original image
					ROI_height = image.rows/6;	//Height of the ROI is 1/6 of the height of the original image.
				}
				not_assigned = false;
			}
			imgROI=image(Rect(0, ROI_y, ROI_width, ROI_height));	//Get the region of interest(ROI)

			//Convert the ROI to gray color
			cvtColor( imgROI, imgROI, CV_BGR2GRAY );
		
			/*
			Smoothing the ROI with GaussianBlur to reduce noise
			The most useful filter (although not the fastest). Gaussian filtering is done by convolving each point in the input array with a Gaussian kernel and then summing them all to produce the output array.
			GaussianBlur(src, dst, Size( 3, 3 ), 0, 0 )
			src: Source image
			dst: Destination image
			Size(w, h): The size of the kernel to be used (the neighbors to be considered). w and h have to be odd and positive numbers otherwise thi size will be calculated using the \sigma_{x} and \sigma_{y} arguments.
			sigma_{x}: The standard deviation in x. Writing 0 implies that \sigma_{x} is calculated using kernel size.
			sigma_{y}: The standard deviation in y. Writing 0 implies that \sigma_{y} is calculated using kernel size.
			*/
			
			GaussianBlur(imgROI, imgROI, Size( 9, 9 ), 2, 2 );
		
			/* 
			Using the threshold to convert the ROI from gray to black & white
			0 for folloeing white line
			1 for following black line
			*/
			threshold(imgROI,imgROI, thresh, 255, is_black);
			
			//morphological opening (remove small objects from the foreground)
			erode(imgROI, imgROI, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
			dilate( imgROI, imgROI, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		
			//morphological closing (fill small holes in the foreground)
			//dilate( imgROI, imgROI, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
			//erode(imgROI, imgROI, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
			
			offset = x_Offset(imgROI, ROI_width);		// Call the x_Offset function to get the offset
			
			move_bot(offset, normal_speed);	//Call the move_bot function which moves the bot with the specified speed
			
			if( show_image )	// If the show_image is 1 then it shows the image and it does not if it is 0
			{
				imshow("Project@Dexter_Industry",imgROI);	// Show the region of interest which is a subset of image in B&W.
				
				if( waitKey(1) == 27)	//	Wait for 1 millisecond, if the user typed "Esc" button from keyboard then the program will get out from this infinite loop
				{
					stop();
					break;
				}
			}
			//usleep(10000);			//sleep for 10 ms
		}
	}
	BrickPiUpdateValues();
	raspiCamCvReleaseCapture(&camera);	// Close the the streaming.
	
	if(show_image)	// If the show_image is 1 then it closes the window 
		destroyWindow("Project@Dexter_Industry");	//Close the window on which images were getting shown.
		
	cout<<"Closing Camera..."<<endl;
	return 1;
}
