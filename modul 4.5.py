#version LOG
#1.1
#using multi threading method with WebcamVideoStream
#so the program that get the image can works on diffirent core
#1.0
#basic stuff

#import imutils
import numpy as np
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import cv2
import time
import serial

#set the resolution
width =  1280 #640 1280
height =  720 #480 720

#set the camera and warming up
cap = WebcamVideoStream(src=0,resolution=(width,height)).start()
time.sleep(2.0)


# Set the port name and the baud rate. This baud rate should match the
# baud rate set on the Arduino.
# Timeout parameter makes sure that program doesn't get stuck if data isn't
# being received. After 1 second, the function will return with whatever data
# it has. The readline() function will only wait 1 second for a complete line 
# of input.

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)

# Get rid of garbage/incomplete data

ser.flush()
time.sleep(3)

#variable
#the minimum matching keypoint to detect the object
MIN_MATCH_COUNT = 12

#degree motor
min_X_degree = 55   #min camera X angle 54.79
max_X_degree = 125  #max camera X angle 125.21
min_Y_degree = 70   #min camera Y angle 68.35
max_Y_degree = 110  #max camera Y angle 111.65

#no_frame=0
i = 0
midpoint = (280,360)

def LIVE_CAM_ORB(live_cam,midpoint):
    img = live_cam

    live_cam = cv2.circle(live_cam,(midpoint[0],midpoint[1]),5,255,-1)
    
    #searching degree for motor value
    motor_degree(midpoint)
        
    fps.update()
    return live_cam

def motor_degree(midpoint):
    #get X degree
    #X_degree = int((midpoint[0]/width*(max_X_degree-min_X_degree)) + min_X_degree)
    X_degree = int(max_X_degree-(midpoint[0]/width*(max_X_degree-min_X_degree)))
    #get Y degree
    Y_degree = int(max_Y_degree-(midpoint[1]/height*(max_Y_degree-min_Y_degree)))
    #Y_degree = int((midpoint[1]/height*(max_Y_degree-min_Y_degree))+min_Y_degree)
    
    #because the servo only can move until 75 degree
    #become 90 coz it hit the camera
    if (Y_degree < 90):
        Y_degree = 90

    #for debugging
    #print( "X,Y = ({},{})".format(midpoint[0],midpoint[1]))
    #print( "X,Y = ({},{})".format(X_degree,Y_degree))
    
    #only send data if the value between the parameter
    if(X_degree<max_X_degree and X_degree>min_X_degree and Y_degree<max_Y_degree and Y_degree>min_Y_degree):
        #sending the value to arduino
        send_to_arduino(X_degree,Y_degree)    
    
def send_to_arduino(x,y):
    angle_value_list = [str(x),str(y)]
    send_string = ','.join(angle_value_list)
    send_string += "\n"

    #send_string ="["
    #send_string += ','.join(angle_value_list)
    #send_string += "]\n"

    #debugging
    #to check the value in send_string
    #print( "send string = {}".format(send_string))
    #print(send_string)
    
    # Send the string. Make sure you encode it before you send it to the Arduino.
    ser.write(send_string.encode('utf-8'))
    #time.sleep(0.2)
    




while(i<1):
    frame = cap.read()
    fps = FPS().start()
   
    frame = LIVE_CAM_ORB(frame,midpoint)
    
    #FPS module
    fps.stop()
    #print(fps.elapsed())
    #print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    #cv2.imwrite('no.' +str(no_frame)+'.jpg',frame)
    #no_frame+=1
    
    cv2.imshow("the actual frame", frame)
    cv2.imwrite(str(midpoint[0])+'x'+str(midpoint[1])+'.jpg',frame)
    
    
    #for debugging
    #receive serial print from arduino for checking value that rasp send
    receive_string = ser.readline().decode('utf-8', 'replace').rstrip()
    #print(receive_string)
    ser.flushInput()
    
    #press enter to exit
    if cv2.waitKey(1) == 13:
        break
    time.sleep(3)
    i=+1

cap.stop()
cv2.destroyAllWindows()

