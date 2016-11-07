#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <raspicam/raspicam_cv.h>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string>
#include "serial.h"
#include <wiringPi.h>
#include <ctime>

#include "ble_read_write.h"

using namespace std;
using namespace cv;


extern int fd;

struct system_status{
    int checksystem;
};

clock_t last_int = clock();
bool toggle_bit=1;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
int counter =0;
int r_c = 0;
int g_c =0;
int o_c=0;
int y_c =0;
int b_c =0;
int p_c =0;
int t_c =0;
int current_detection =0;
pthread_mutex_t img_mutex;
pthread_mutex_t ble_mutex;
Mat img;
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
		p_c+=1;
                break;
            case 2:
                cout<<"yellow is detected"<<endl;
		current_detection =2;
		y_c +=1;
                break;
            case 3:
                cout<<"red is detected"<<endl;
		current_detection =3;
                r_c+=1;
		break;
            case 4:
                cout<<"orange is detected"<<endl;
		current_detection =3;
		o_c+=1;
                break;
            case 5:
                cout<<"green is detected"<<endl;
		current_detection =5;
		g_c+=1;
                break;
            case 6:
                cout<<"blue is detected"<<endl;
		current_detection =4;
		b_c +=1;
                break;
            default:
		current_detection =0;
                cout<<"nothing is detected"<<endl;
        }
        //cout<<"number detected:" <<max_count<<endl;

}

void color_detection(){

    //check the time interval between two interrupt
    double duration;
    clock_t current = clock();
    duration = (current-last_int)/(double)CLOCKS_PER_SEC;
    if(duration < 1){
        last_int = current;
	return;
    }
    last_int = current;
    
    
    Mat seg(img.rows, img.cols, CV_8UC1);
    Mat color_seg(img.rows, img.cols, CV_8UC3);
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

    pthread_mutex_lock(&img_mutex);

    GaussianBlur(img,img, Size(5,5), 0,0);
    for (int y =0; y <img.rows; y++){
        for (int x =0; x < img.cols; x++){

            Vec3b color = img.at<Vec3b>(y,x);
            uchar red = color.val[2];
            uchar green = color.val[1];
            uchar blue = color.val[0];
            if(red-green > 50 &&red > 140 && green <180 && blue <200 && blue>85){
                color_seg.at<Vec3b>(y,x)[0] = 203;
                color_seg.at<Vec3b>(y,x)[1] = 192;
                color_seg.at<Vec3b>(y,x)[2] = 255;
                pink_counter ++;

            }
            else if(red > 110 && green >100 && red-green< 50 && red-green >0 && blue <100){
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
            else if(red> 20 && green-red >50 && blue <80){
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
    pthread_mutex_unlock(&img_mutex);

        stringstream name;
    name <<"image_"<<t_c<<".jpg";
    imwrite(name.str(), img);


	
    count_array[0] = pink_counter;
    count_array[1] = yellow_counter;
    count_array[2] = red_counter;
    count_array[3] = orange_counter;
    count_array[4] = green_counter;
    count_array[5] = blue_counter;
    
    find_maximum(count_array,6);

    //send message over serial
    printf("color_detect is called %d\n", counter);
    char message[2];
    message[0] = (char)current_detection;
    message[1] = '\0';
    serial_send(message);
    
    printf("serial sent color: %d\n", current_detection);
    t_c+=1;

    //send over ble
    uint8_t send[1];
    toggle_bit= !toggle_bit;
    cout<<"toggle bit: "<< toggle_bit<<endl;
    if(toggle_bit)
    	send[0] = (uint8_t)(current_detection | (1<<3)) ;
    else
	send[0] = (uint8_t) current_detection;

    //delay
    usleep(30000);
    //send[1] = 0xA5;
    pthread_mutex_lock(&ble_mutex);
    ble_write(send);
    pthread_mutex_unlock(&ble_mutex);

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


void notification_cb(uint16_t handle, const uint8_t* data, size_t data_length, void* user_data){
        struct system_status* s1 = (struct system_status *)(user_data);
        s1->checksystem = 1;
        printf("Notification on handle %d\n", handle);
//      *user_data = PtrToOne[0];

}

int main(){
    serial_init();
    pinMode(3,OUTPUT);
    ble_init();
    Mat image;
    struct system_status s;
    s.checksystem = 0;

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

    Camera.grab();
    Camera.retrieve(image);
    img = image(cv::Rect(380, 460, 400, 500));
    if(interrupt_init(&color_detection) ==1){
	cerr<<"Unable to set up"<<endl;
	return 1;
    }

    char key =0;
    int counter_for_ble = 0;
    uint8_t result = 5;

    if(1){
    //register notification
    printf("status notif\n");
    uint16_t status_handle = 0x002f;
    uint16_t enable_notif = 0x0001;
    // Enable status notif
    gattlib_write_char_by_handle(connection, status_handle+1, &enable_notif, sizeof(enable_notif));
    printf("register\n");
    // register notif handler
    gattlib_register_notification(connection, notification_cb, &s);
    }


    while( key != 27){
        Camera.grab();
    	pthread_mutex_lock(&img_mutex);
        Camera.retrieve(image);
	//Mat sub_img = image(cv::Rect(380, 460, 400, 500));
	//Mat color_seg (sub_img.rows, sub_img.cols, CV_8UC3);
	img = image(cv::Rect(380, 460, 400, 500));
	


	//color_detection();
	key = waitKey(50);


	imshow("color", img);
	pthread_mutex_unlock(&img_mutex);
	//read ble
	//if(1){
	if(s.checksystem == 1){
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
	    printf("start check system\n");
	    usleep(100000);
	    
	    pthread_mutex_lock(&ble_mutex);
	    result = ble_read();
	    pthread_mutex_unlock(&ble_mutex);
	    s.checksystem = 0;
	    printf("check system\n");
	    result = result & 0x7;
            if(result == 4){
	       //if stop pulse gpio to INT
		printf("send pulse to stop motor\n");
		digitalWrite(3,1);
		usleep(1000);
		digitalWrite(3,0);
	    }
	    if(result ==5){
    		printf("send restart message to ble\n");
    		char message[4]="run";
    		serial_send(message);
	    }

	}
	usleep(10000);

	//pthread_mutex_unlock(&img_mutex);
	

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
	if(key == 99){
	    imwrite("image1.jpg", image);
	}
	//counter_for_ble++;
   }
    
    Camera.release();
    cvDestroyWindow( "result" );
    cvDestroyWindow("color");
    return 0;
	
}

