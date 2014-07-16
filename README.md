open_cv_pi
==========

The project name Line Follower Pi is an opencv project. In this project we are using Raspberry Pi. 
The project is for making an autonomous line follower bot using the technique of image processing.
This can track white and black line. There is an argument which let you chose which colored line you want to follow.
Since it is just an introduction, i cannot give all the technical details in this parafraph. So stay along with me to have your own Line Follower Pi.
Though it is a basic project in opencv but for Raspberry Pi user it will be a good start to opencv.
No need to worry about it. Even you are a beginner to opencv. 
I will guide you from the installation of opencv on your Pi to the completion of this project.

Components required for this project are:
	1> Raspberry Pi
	2> Raspberry Pi camera
	3> Brick Pi ( for controlling motors )
	4> Two lego motors
	5> Wifi adapter 
	6> Wifi connection
	7> 8 GB SD card
	8> Power cable for your Raspberry Pi
	9> Preferably a new case for your Raspberry Pi and Brick Pi, which is recently designed by me. Just for the safety of your Raspberry Pi and Brick Pi. 
	
Now let us build your own Line Follower Pi in following simple steps :

	STEP 1: Installation of opencv on your Raspberry Pi - We will install opencv on the Raspberry Pi. I have documented the installation guide for opencv on Raspberry Pi
				Here is the link to the installatin guide for opencv on Raspberry Pi : https://docs.google.com/document/d/1op8RSzmRqjiwh9KCkuavni5GiDaExN0I9GA6pvZy1EI/edit
				With the help of that document you will be able to run some simple opencv program with and without using Raspberry Pi camera.

	STEP 2:  Making this program to run on your Pi - We all know every color is made of 3 primary color namely Red(R), Green(G) and Blue(B). In short RGB.
				In opencv image captured from camera will be in BGR form means Blue, Green and Red. You might have think that I have just changed the order of color.
				But it is not like that. In opencv the order by which we specify the color is dependent on the image matrix.
				There are many color spaces other than RGB like HSV, GRAY etc. In this project we will be working with Gray image.
				Does it sound strange? Camera gives BGR format image then, how do I convert this image to Gray color image?
				No, it is very simple. In opencv it is very simple to convert the BGR format image to Gray color image.
				The gray color image is a single channel image i.e. only one 2D matrix. In this matrix elements can have value from 0 to 255.
				0 represents completly blacck and 255 represents completely white.
				In this program all you have to do is run the program with 3 arguments
				
				First one is threshold which signifies the amount of whiteness or blackness. If trying to track white line then try to keep it higher.
				If trying to track the black line then try to keep it lower.This ranges from 0 to 255.
				
				Second argument is the speed with which you would like to move your Line Follower Pi. Try to use the smaller value like near 100. So that overshoot does not take place.
				Here overshoot means when you runs the Line Follower Pi at higher speed at the turning point it moves ahead instead of taking a turn. This results in loss of line from the 
				field of view of your Raspberry Pi camera. This ranges from 0 to 255.
				
				Third argument is just to say your Line Follower Pi that which line to detect. 0 is to detect white line and 1 is to detect black line.

				There are few more arguments but these are optional. ROI is stands for region of interest. Here it is like a rectangle or a 2D box.
				
				Fourth argument is to change the y coordinate of the origin of the ROI.
				
				Fifth argument is the height of this rectangle.
				
				Press Ctrl+C in terminal to stop the program. 
	Since there are four source code, So I would prefer you to use the latest one for the best result. Latest code will be having greater number in the name of the code.			
	That's all for this project. Congratulation! now you have your own Line Follower Pi with you.
	Have a great fun with your Line Follower Pi.
	Feel free to contribute if you are interested to develope this to a next level.