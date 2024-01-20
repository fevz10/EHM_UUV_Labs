#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <pthread.h>


#define MOTOR_CTRL_CAN_ID 0x400
#define MOTOR_CTRL_CAN_DLC 8
#define ESC_KEY 27

using namespace std;
using namespace cv;

int can_socket; 


void *thread(void *)
{
    struct can_frame rxmsg;      
    while (1)
    {   
        read(can_socket, &rxmsg, sizeof(rxmsg));
        printf("message received\n");                 
    }
}

int main(void)
{
    VideoCapture webcamDevice(0);
    Mat webcamFrame, frame, hsv, mask, c, gauss;
    int frame_x, frame_y;
    vector<Vec4i> hierarchy;
    Rect boundRect;
    vector<vector<Point>> cnts;

	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame canFrame;
    uint8_t motor1_pwm1=0, motor1_pwm2=0, motor2_pwm1=0, motor2_pwm2=0, motor3_pwm1=0, motor3_pwm2=0, motor4_pwm1=0, motor4_pwm2=0;
    int min_area, x_point, y_point;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
		cout << "Failed to open CAN port initialize\n";
		return -1;
	}
    strcpy(ifr.ifr_name, "vcan0" );
	ioctl(can_socket, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
		cout << "CAN port open succesfully\n";
		return -1;
	}

    pthread_t pth;
    pthread_create(&pth, NULL, thread, NULL);


    if ( !webcamDevice.isOpened() )
    {
        cout << "No webcam detected!\n";
        return -1;
    }

    while(true)
    {
        webcamDevice >> webcamFrame;
        if ( webcamFrame.empty() )
        {
            break;
        }
        resize(webcamFrame, frame, Size(640, 480), INTER_LINEAR);
        GaussianBlur(frame, gauss, Size(5, 5), 0);
        cvtColor(gauss, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 101, 221), Scalar(179, 172, 255), mask);
        findContours(mask, cnts, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );

        frame_x = frame.size().width;
        frame_y = frame.size().height;

        size_t length = cnts.size();

        if(length > 0)
        {
            auto maxContour = std::max_element(cnts.begin(), cnts.end(),
                [](const auto& a, const auto& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
            });

            if (maxContour != cnts.end())
            {
                // Access the contour with the maximum area
                vector<Point> maxContourPoints = *maxContour;
                boundRect = boundingRect(maxContourPoints);
                int x = boundRect.x;
                int y = boundRect.y;
                int w = boundRect.width;
                int h = boundRect.height;
                rectangle(frame, Point(x,y), Point(x+w,y+h), Scalar(0,255,0), 3);
                putText(frame,"Ball",cv::Point(x+w-20,y+h+30),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),1,false);
            
                min_area = w * h;
                x_point = x + (w/2);
                y_point = y + (h/2);

                if ( (min_area > 700) && (min_area < 25000) )
                {
                    if ( x_point < ((frame_y/2)+(frame_y/3)) )
                    {
                        cout << "Right turn\n";
                        motor1_pwm1 = 50;
                        motor1_pwm2 = 0;

                        motor2_pwm1 = 0;
                        motor2_pwm2 = 0;

                        motor3_pwm1 = 50;
                        motor3_pwm2 = 0;

                        motor4_pwm1 = 0;
                        motor4_pwm2 = 0;
                        
                    }
                    else if ( x_point < ((frame_y/2)-(frame_y/3)) )
                    {
                        cout << "Left turn\n";
                        motor1_pwm1 = 0;
                        motor1_pwm2 = 0;

                        motor2_pwm1 = 50;
                        motor2_pwm2 = 0;

                        motor3_pwm1 = 0;
                        motor3_pwm2 = 0;

                        motor4_pwm1 = 50;
                        motor4_pwm2 = 0;
                    }
                    else
                    {
                        cout << "Forward\n";
                        motor1_pwm1 = 50;
                        motor1_pwm2 = 0;

                        motor2_pwm1 = 50;
                        motor2_pwm2 = 0;

                        motor3_pwm1 = 50;
                        motor3_pwm2 = 0;

                        motor4_pwm1 = 50;
                        motor4_pwm2 = 0;

                    }
                }
                else if ( (min_area > 300) && (min_area < 700) )
                {
                    cout << "Forward\n";
                    motor1_pwm1 = 50;
                    motor1_pwm2 = 0;

                    motor2_pwm1 = 50;
                    motor2_pwm2 = 0;

                    motor3_pwm1 = 50;
                    motor3_pwm2 = 0;

                    motor4_pwm1 = 50;
                    motor4_pwm2 = 0;
                }
            }
        }
        else
        {
            std::cout << "Right turn\n";
            motor1_pwm1 = 50;
            motor1_pwm2 = 0;

            motor2_pwm1 = 0;
            motor2_pwm2 = 0;

            motor3_pwm1 = 50;
            motor3_pwm2 = 0;

            motor4_pwm1 = 0;
            motor4_pwm2 = 0;
        }

        imshow("lab3", frame);

        canFrame.can_id = MOTOR_CTRL_CAN_ID;
        canFrame.can_dlc = MOTOR_CTRL_CAN_DLC;
        canFrame.data[0] = motor1_pwm1;
        canFrame.data[1] = motor1_pwm2;
        canFrame.data[2] = motor2_pwm1;
        canFrame.data[3] = motor2_pwm2;
        canFrame.data[4] = motor3_pwm1;
        canFrame.data[5] = motor3_pwm2;
        canFrame.data[6] = motor4_pwm1;
        canFrame.data[7] = motor4_pwm2;        

        write(can_socket, &canFrame, sizeof(struct can_frame));

        if ( ((char)waitKey(20)) == ESC_KEY )
        {
            break;
        }
    }

    close(can_socket);
    webcamDevice.release();
    return 0;
}