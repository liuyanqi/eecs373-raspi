#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <raspicam/raspicam_cv.h>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "serial.h"

using namespace std;
using namespace cv;

extern int fd;

int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
int counter =0;

int current_detection =0;

void find_maximum(int count_array[6], int size){
        int max_count = 0;
        int idx =0;
        for (int i =0; i< size; i++){
            if(count_array[i] > max_count){
                max_count = count_array[i];
                idx = i+1;
            }
        }
        switch(idx){

            case 1:
                cout<<"pink is detected"<<endl;
		current_detection =1;
                break;
            case 2:
                cout<<"yellow is detected"<<endl;
		current_detection =2;
                break;
            case 3:
                cout<<"red is detected"<<endl;
		current_detection =3;
                break;
            case 4:
                cout<<"orange is detected"<<endl;
		current_detection =4;
                break;
            case 5:
                cout<<"green is detected"<<endl;
		current_detection =5;
                break;
            case 6:
                cout<<"blue is detected"<<endl;
		current_detection =6;
                break;
            default:
		current_detection =0;
                cout<<"nothing is detected"<<endl;
        }
        //cout<<"number detected:" <<max_count<<endl;

}

void color_detection(Mat& img, Mat& color_seg){
    GaussianBlur(img,img, Size(71,71), 0,0);
    GaussianBlur(img,img, Size(9,9), 0,0);

    
    Mat seg(img.rows, img.cols, CV_8UC1);
    Mat seg2;

    int pink_counter =0;
    int yellow_counter =0;
    int red_counter =0;
    int orange_counter =0;
    int green_counter =0;
    int blue_counter =0;

    int count_array[6];
    Scalar pink_low(0,0,100);
    Scalar pink_high(100,100,255);
    // inRange(img,pink_low, pink_high,seg2);


    for (int y =0; y <img.rows; y++){
        for (int x =0; x < img.cols; x++){
      
            Vec3b color = img.at<Vec3b>(y,x);
            uchar red = color.val[2];
            uchar green = color.val[1];
            uchar blue = color.val[0];
            if(red-green > 50 &&red > 140 && green <180 && blue <200 && blue>100){
                color_seg.at<Vec3b>(y,x)[0] = 203;
                color_seg.at<Vec3b>(y,x)[1] = 192;
                color_seg.at<Vec3b>(y,x)[2] = 255;
                pink_counter ++;

            }
            else if(red > 150 && green >100 &&red >green && red-green< 50 && blue <100){
                color_seg.at<Vec3b>(y,x)[0] = 0;
                color_seg.at<Vec3b>(y,x)[1] = 255;
                color_seg.at<Vec3b>(y,x)[2] = 255;
                yellow_counter ++;

            }

            else if(red > 100 && red-green > 50 && red-blue >50 && green <100 && blue<100){
                // seg.at<Vec3b>(y,x) = Scalar(0,0,255);
                color_seg.at<Vec3b>(y,x)[0] = 0;
                color_seg.at<Vec3b>(y,x)[1] = 0;
                color_seg.at<Vec3b>(y,x)[2] = 255;
                red_counter ++;

            }
            else if(red > 200 && green > 110 && green < 160 && blue <100){
                color_seg.at<Vec3b>(y,x)[0] = 0;
                color_seg.at<Vec3b>(y,x)[1] = 165;
                color_seg.at<Vec3b>(y,x)[2] = 255;
                orange_counter++;

            }
            else if(red> 100 && green >200 && blue <30){
                color_seg.at<Vec3b>(y,x)[0] = 1;
                color_seg.at<Vec3b>(y,x)[1] = 255;
                color_seg.at<Vec3b>(y,x)[2] = 1;
                green_counter++;

            }
            else if(red < 110 && green < 110 && blue > 100){
                color_seg.at<Vec3b>(y,x)[0] = 255;
                color_seg.at<Vec3b>(y,x)[1] = 0;
                color_seg.at<Vec3b>(y,x)[2] = 0;
                blue_counter++;

            }
            else{
                seg.at<uchar>(y,x) =255;
                color_seg.at<Vec3b>(y,x)[0] =255;
                color_seg.at<Vec3b>(y,x)[1] = 255;
                color_seg.at<Vec3b>(y,x)[2] = 255;

            }
        }
    }
    count_array[0] = pink_counter;
    count_array[1] = yellow_counter;
    count_array[2] = red_counter;
    count_array[3] = orange_counter;
    count_array[4] = green_counter;
    count_array[5] = blue_counter;
    
    find_maximum(count_array,6);
   // cout<<"pink_counter: " << pink_counter << "yellow_counter: "<<yellow_counter <<"red_counter: "<<red_counter << "orange_counter: " << orange_counter<<
   //  "green_counter: "<<green_counter <<"blue_counter: " << blue_counter<<endl;


}

void myInterrupt2(void){
	printf("interrupt2 is called %d\n", counter);
	char message[2];
	message[0] = (char)current_detection;
	message[1] = '\0';
	serial_send(message);
	printf("serial sent color: %d\n", current_detection);
	counter++;
}

int main(){
    serial_init();
    if(fd < 0){
	cerr<<"Error opening wiringpi"<<endl;
	return 1;
    }


    raspicam::RaspiCam_Cv Camera;
    Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    cout<<"Opening Camera..."<<endl;
    if(!Camera.open()){
	cerr<<"Error opening the camera"<<endl;
	return -1;
     }

    if(interrupt_init(&myInterrupt2) ==1){
	cerr<<"Unable to set up"<<endl;
	return 1;
    }

    char key =0;
    Mat image;
    while( key != 27){
        Camera.grab();
        Camera.retrieve(image);
	Mat sub_img = image(cv::Rect(380, 460, 400, 500));
	Mat color_seg (sub_img.rows, sub_img.cols, CV_8UC3);

	//cout<<sub_img.rows <<" "<<sub_img.cols<<endl;
	color_detection(sub_img, color_seg);
	key = waitKey(50);
        imshow("result", color_seg);
	imshow("color", sub_img);

	if(key == 115) {//key=='s'
	    char message[6];
	    message[0] = 'H';
	    message[1] = 'E';
	    message[2] = 'L';
	    message[3] = 'L';
	    message[4] = 'O';
	    message[5] = '\0';
	    serial_send(message);
	   cout<<"serial sent"<<endl;
	}	
   }
    
    Camera.release();
    cvDestroyWindow( "result" );
    cvDestroyWindow("color");
    return 0;
	
}

