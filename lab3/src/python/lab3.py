import can
import cv2
import numpy as np
import time
import threading

motor1_pwm1 = 0
motor1_pwm2 = 0

motor2_pwm1 = 0
motor2_pwm2 = 0

motor3_pwm1 = 0
motor3_pwm2 = 0

motor4_pwm1 = 0
motor4_pwm2 = 0

class CAN(object):

    def __init__(self,port='can0',ID=1024,bitrate=500000):
        self.port = port
        self.bitrate = bitrate
        self.bustype = "socketcan"
        self.ID = ID
        self.tx_buffer = [0,0,0,0,0,0,0,0]
        self.period = 100
    
    def read(self):
        while(True):
            bus = can.interface.Bus(self.port,bustype=self.bustype)
            message = bus.recv()

    def write(self):
        while(True):
            bus = can.interface.Bus(channel=self.port,bustype=self.bustype)
            msg = can.Message(arbitration_id = self.ID,data=self.tx_buffer, is_extended_id=False, dlc=8)
            bus.send(msg)
            time.sleep(self.period/1000)

    def run(self):
        threading.Thread(target = self.write).start()


cap = cv2.VideoCapture(0)
canbus = CAN()
canbus.run()
Lower = (0, 101, 221)
Upper = (179, 172, 255)
kernel = np.ones((3,3), np.uint8); # for mask
font = cv2.FONT_HERSHEY_SIMPLEX 

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.resize(frame, (640,480))
    frame = cv2.GaussianBlur(frame, (5,5), 0) #smoothing
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, Lower, Upper)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_x, frame_y, _ = frame.shape 
    min_area = 0; # min rect. shape w*h
    x_point = 0; # frame center
    y_point = 0;   #frame center 
    
    #if camera find the ball
    
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea) #maximum contour
        (x, y, w, h) = cv2.boundingRect(c) 
        #x and y starting point, w and h height and width ratio
        cv2.rectangle(frame, (x,y),(x+w,y+h), (0,255,0), 3) # min rectangular drawing
        frame = cv2.putText(frame, 'Ball', (x+w-20,y+h+30), font, 1, (255,0,255), 1, cv2.LINE_AA)        
        min_area = w*h #rectangular area
        x_point = x + (w/2) # object center
        y_point = y + (h/2)
        
        if((min_area > 700) and (min_area < 25000)):
            if( x_point > ((frame_y/2) + (frame_y/3)) ):
                #print("Right turn")
                motor1_pwm1 = 50
                motor1_pwm2 = 0
                motor2_pwm1 = 0
                motor2_pwm2 = 0
                motor3_pwm1 = 50
                motor3_pwm2 = 0
                motor4_pwm1 = 0
                motor4_pwm2 = 0
            elif( x_point < ((frame_y/2) - (frame_y/3)) ):
                #print("Left turn")
                motor1_pwm1 = 0
                motor1_pwm2 = 0
                motor2_pwm1 = 50
                motor2_pwm2 = 0
                motor3_pwm1 = 0
                motor3_pwm2 = 0
                motor4_pwm1 = 50
                motor4_pwm2 = 0
            else:
                #print("Forward")
                motor1_pwm1 = 50
                motor1_pwm2 = 0
                motor2_pwm1 = 50
                motor2_pwm2 = 0
                motor3_pwm1 = 50
                motor3_pwm2 = 0
                motor4_pwm1 = 50
                motor4_pwm2 = 0
        elif((min_area > 300) and (min_area < 700)):
            #print("Forward")
            motor1_pwm1 = 50
            motor1_pwm2 = 0
            motor2_pwm1 = 50
            motor2_pwm2 = 0
            motor3_pwm1 = 50
            motor3_pwm2 = 0
            motor4_pwm1 = 50
            motor4_pwm2 = 0
    else:
        #print("Right turn")
        motor1_pwm1 = 50
        motor1_pwm2 = 0
        motor2_pwm1 = 0
        motor2_pwm2 = 0
        motor3_pwm1 = 50
        motor3_pwm2 = 0
        motor4_pwm1 = 0
        motor4_pwm2 = 0
        
    cv2.imshow("lab3", frame)
    canbus.tx_buffer = [motor1_pwm1, motor1_pwm2, motor2_pwm1, motor2_pwm2, motor3_pwm1, motor3_pwm2, motor4_pwm1, motor4_pwm2]
    if cv2.waitKey(1) & 0xFF == ord("q"):      
        break

cv2.destroyAllWindows()