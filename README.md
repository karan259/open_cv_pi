Line Follower Pi
==========

The project name is Line Follower Pi. The software will help you in tracking colored ball. It is a basic opencv project. In this project we will be using Raspberry Pi. The project is for making an autonomous line follower bot using the technique of image processing. The bot can track white line in balck background and black line in white background. This will guide you from the installation of opencv on your Pi to the completion of this project.

Components required for this project are:
- Raspberry Pi
- Raspberry Pi camera
- Brick Pi ( for controlling motors )
- Two lego motors
- Wifi adapter 
- Wifi connection
- 8 GB SD card
- Power cable for your Raspberry Pi
- For safety of the Raspberry Pi, user may prefer the case for Raspberry Pi and Brick Pi
	
Now let us build your own Line Follower Pi in following simple steps

1. **Installation of opencv on your Raspberry Pi** - We will install opencv on the Raspberry Pi. Here is a guide for the installation of opencv on Raspberry Pi. Here is the link to the installatin guide for opencv on Raspberry Pi : https://docs.google.com/document/d/1op8RSzmRqjiwh9KCkuavni5GiDaExN0I9GA6pvZy1EI/edit With this document. Installation will be easier. Can be able to run some simple opencv program with and without using Raspberry Pi camera. 

**General Information about the following steps** 

To run this program on your Pi. Every color is made of 3 primary color namely Red(R), Green(G) and Blue(B). In short RGB. In opencv image captured from camera will be in BGR form means Blue, Green and Red. This might seeems that it just a change in the order of color. But it is not like that. In opencv the order by which we specify the color is dependent on the image matrix.

In opencv there are many color spaces other than RGB like HSV, GRAY etc. In this project we will be using Gray color space.

In opencv it is very simple to convert the BGR format image to Gray color image.
The gray color image is a single channel image i.e. only one 2D matrix. In this matrix elements can have value from 0 to 255.0 represents completly blacck and 255 represents completely white.
    
2. **Making this program to run on your Pi** - First have to compile the source code. After the compilation we will run the program with few arguments. *See the end of this ReadMe file for compilation and running of the source code*.

*Arguments for this steps*

1. **threshold** - which signifies the amount of whiteness or blackness. If trying to track white line then try to keep it higher. If trying to track the black line then try to keep it lower.This ranges from 0 to 255.

2. **speed** - with which you would like to move your Line Follower Pi. Try to use the smaller value like near 100. So that overshoot does not take place.

Here overshoot means when you runs the Line Follower Pi at higher speed at the turning point it moves ahead instead of taking a turn. This results in loss of line from the field of view of your Raspberry Pi camera. This ranges from 0 to 255.

3. **black** - signifies the color of the line. 0 is to track white line and 1 is to track black line.

There are few more arguments but those are optional. ROI is stands for region of interest. ROI is like a rectangle or a 2D box. For better understanding of the ROI, we can think it as a rectangle. Having the width equal to width of the the original image. We can shift this in vertical direction by specifying ROI_y. By default it is at 1/3 times the y-axis of the original image. Since y-axis is constrained by the number of rows in the image matrix. By default height is equal to 1/6 times of the height of the image matrix.

4. **ROI_y** - this argument is to change the y coordinate of the origin of the ROI. Give it in percentage like if we want to shift the ROI by 30 percent from top then give it as 30. Similarly to shift the ROI by 50 percent in vertical direction give it as 50. Take care of the height of the ROI. As you shift the ROI down 30 percent, then we can increse the height up to  the maximum of 70 percent.  

5. **ROI_height** - this is the height of this rectangle. Give this in percentage as explained above.
				
Press Ctrl+C in terminal to stop the program. 

Since there are four source code, So I would prefer you to use the latest one for the best result. Latest code will be having greater number in the name of the code.			
That's all for this project. Now you have your own Line Follower Pi.

**How to compile**

```C++
g++ `pkg-config --cflags opencv` line_folower_2.cpp -o test_3 `pkg-config --libs opencv` -I/home/pi/git/robidouille/raspicam_cv -L/home/pi/git/robidouille/raspicam_cv -lraspicamcv -L/home/pi/git/raspberrypi/userland/build/lib -lmmal_core -lmmal -l mmal_util -lvcos -lbcm_host -lrt -lm -L/usr/local/lib -lwiringPi
```
 
**How to run**
 
```C++
./test_2 threshold speed black
or using optional arguments to change the ROI property
./test_2 threshold speed black ROI_y ROI_height
```