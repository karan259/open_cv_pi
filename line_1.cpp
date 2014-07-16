//new code
/*
########################################################################                                                                 
# Program Name: simplebot_speed.c                                     
# ================================     
# This code is for moving the simplebot  with speed control                                  
# http://www.dexterindustries.com/                                                                
# History
# ------------------------------------------------
# Author     Date      Comments
# Govinda      06/11/14  Initial Authoring
#                                                                  
# These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
# (http://creativecommons.org/licenses/by-sa/3.0/)           
#
########################################################################
*/
//Command:
//	Esc - To stop the program

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

int thresh;
Mat image;
Mat imgROI;
float offset;
int max_speed;

// gcc -o program simplebot_simple.c -lrt -lm -L/usr/local/lib -lwiringPi
// ./program

int result,speed_1,speed_2;//Set the speed
int motor1,motor2;
//char cmd;

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

void x_Offset()
{
	
	imgROI = image(Rect(0,image.rows/3,image.cols,image.rows/6));
	cvtColor( image, imgROI, CV_BGR2GRAY );
	GaussianBlur(imgROI, imgROI, Size( 9, 9 ), 2, 2 );
	threshold(imgROI,imgROI, thresh, 255, 0);

	Moments mu;
	mu = moments(imgROI, true);
	Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
	offset = (1-2*center.x/image.cols);

	if(isnan(offset)) 
		offset = 0;

	//y_offset = (1-center.y/image.rows);
	cout<<offset<<"\n"<<flush;
}

void move_bot()
{
	if(fabs(offset)>0.3)
	{
		if(offset>0)
		{	speed_1=max_speed+max_speed*0.1;speed_2=max_speed+max_speed*0.1;
			//speed_1 += 50;
			//speed_2 += (speed_2 * (1-x_offset)*gain);
			//for(int i = 0;i<2;i++)
			left();	
			BrickPiUpdateValues();	//Update the motor values
		}
		else
		{	speed_1=max_speed+max_speed*0.2;speed_2=max_speed+max_speed*0.2;
			//speed_1 += (speed_1 * (-x_offset)*gain);
			//speed_2 +=50;
			//for(int i = 0;i<2;i++)
			right();
			BrickPiUpdateValues();	//Update the motor values
		}
	}
	else if(fabs(offset)>0.5)
	{
		if(offset>0)
		{	speed_1=max_speed+max_speed*0.2;speed_2=max_speed+max_speed*0.2;
			//speed_1 += 50;
			//speed_2 += (speed_2 * (1-x_offset)*gain);
			//for(int i = 0;i<2;i++)
			left();	
			BrickPiUpdateValues();	//Update the motor values
		}
		else
		{	speed_1=max_speed+max_speed*0.2;speed_2=max_speed+max_speed*0.2;
			//speed_1 += (speed_1 * (-x_offset)*gain);
			//speed_2 +=50;
			//for(int i = 0;i<2;i++)
			right();
			BrickPiUpdateValues();	//Update the motor values
		}
	}
	else if(fabs(offset)>0.7)
	{
		if(offset>0)
		{	speed_1=max_speed+max_speed*0.3;speed_2=max_speed+max_speed*0.3;
			//speed_1 += 50;
			//speed_2 += (speed_2 * (1-x_offset)*gain);
			//for(int i = 0;i<2;i++)
				left();	
			BrickPiUpdateValues();	//Update the motor values
		}
		else
		{	speed_1=max_speed+max_speed*0.3;speed_2=max_speed+max_speed*0.3;
			//speed_1 += (speed_1 * (-x_offset)*gain);
			//speed_2 +=50;
			//for(int i = 0;i<2;i++)
				right();
			BrickPiUpdateValues();	//Update the motor values
		
		}	
	}
	else if(fabs(offset)>0.9)
	{
		if(offset>0)
		{	speed_1=max_speed+max_speed*0.4;speed_2=max_speed+max_speed*0.4;
			//speed_1 += 50;
			//speed_2 += (speed_2 * (1-x_offset)*gain);
			//for(int i = 0;i<2;i++)
			left();	
			BrickPiUpdateValues();	//Update the motor values
		}
		else
		{	speed_1=max_speed+max_speed*0.4;speed_2=max_speed+max_speed*0.4;
			//speed_1 += (speed_1 * (-x_offset)*gain);
			//speed_2 +=50;
			//for(int i = 0;i<2;i++)
			right();
			BrickPiUpdateValues();	//Update the motor values
		}	
	}
	else{
		speed_1=speed_2=max_speed;
		fwd();
		BrickPiUpdateValues();	//Update the motor values
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

	BrickPi.Timeout=1000;				//Set timeout value for the time till which to run the motors after the last command is pressed
	BrickPiSetTimeout();				//Set the timeout

	// OpenCV Part Srarts
	thresh = atoi(argv[1]);
	max_speed= atoi(argv[2]);
	
	//int gain = atoi(argv[2]);

	RaspiCamCvCapture* camera = raspiCamCvCreateCameraCapture(0);
    	
	namedWindow( "Project@Dexter_Industry", 0 );
    	cout<<"Opening Camera..."<<endl;
	//Check  for streaming
	if ( !camera ) { 
		cerr<<"Error opening the camera"<< endl;return -1; 
	}
	
	if(!result)
	{
		while(true)
		{	
			image = raspiCamCvQueryFrame( camera );

			x_Offset();
			//cout<<offset<<endl<<flush;
			move_bot();			
	
			imshow("Project@Dexter_Industry",imgROI);
	        	//if ( i%5==0 )  cout<<"\r captured "<<i<<" images"<<std::flush;
			
			if( waitKey(1) == 27)	// If Esc has pressed
			{
				stop();
				break;
			}
			
			//usleep(10000);			//sleep for 10 ms
			
		}
	}
	BrickPiUpdateValues();
	raspiCamCvReleaseCapture(&camera);
	destroyWindow("Project@Dexter_Industry");
	cout<<"Closing Camera..."<<endl;
	return 1;
}
