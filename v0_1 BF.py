import numpy as np
import cv2
import time
import serial

cap = cv2.VideoCapture(0)
#set the resolution
width =  640 #640 1280
height =  480 #480 720

#changing the resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)

# Initiate ORB detector with max 1000 features and scaling 1.2
orb = cv2.ORB_create(1000, 1.2)

#input the reference image
input_image = cv2.imread('raspberry pi.jpg')
input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
#search the keypoint of input image
kp1, des1 = orb.detectAndCompute(input_image,None)
#search the height and Width of the input image
#to make the boundary on display
h,w = input_image.shape
pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

# create BFMatcher object
bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

# Set the port name and the baud rate. This baud rate should match the
# baud rate set on the Arduino.
# Timeout parameter makes sure that program doesn't get stuck if data isn't
# being received. After 1 second, the function will return with whatever data
# it has. The readline() function will only wait 1 second for a complete line 
# of input.
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Get rid of garbage/incomplete data
ser.flush()

#variable
#the minimum matching keypoint to detect the object
MIN_MATCH_COUNT = 12

#degree motor
min_X_degree = 60
max_X_degree = 120
min_Y_degree = 60
max_Y_degree = 120


def LIVE_CAM_ORB(live_cam):
    img = live_cam

    #change the camera image and input image to black and white
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #search the keypoint of live cam
    kp2, des2 = orb.detectAndCompute(img,None)

    # Match descriptors.
    matches = bf.knnMatch(des1,des2,k=2)

    # Apply ratio test
    good = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            good.append(m)

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        #(for debugging)
        #matchesMask = mask.ravel().tolist() 
        dst = cv2.perspectiveTransform(pts,M)
        (tl, tr, br, bl) = dst
        midpoint = (tl+bl+br+tr)/4
        midpoint = (tl[0]+bl[0]+br[0]+tr[0])/4
        #midpoint = np.rint(midpoint) //round the integer
        midpoint = midpoint.astype(int)
        live_cam = cv2.polylines(live_cam,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        live_cam = cv2.circle(live_cam,(midpoint[0],midpoint[1]),5,255,-1)
        
        #for img3 (debugging with black and white)
        #img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        
        #searching degree for motor value
        motor_degree(midpoint)
        
    #else:
        #print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        #(for debugging)
        #matchesMask = None

    #Draw matches. (for debugging)
    #draw_params = dict(matchColor = (0,255,0), # draw matches in green color
    #               singlePointColor = None,
    #               matchesMask = matchesMask, # draw only inliers
    #               flags = 2)

    #img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
    #cv2.imshow("ORB Keypoints and matched", img3)

    return live_cam

def motor_degree(midpoint):
    #get X degree
    X_degree = int((midpoint[0]/width*(max_X_degree-min_X_degree))+min_X_degree)

    #get Y degree
    #because you got a flipped value
    #(the higher you get, the smaller the midpoint value)
    #the formula is different from getting X
    Y_degree = int(max_Y_degree-(midpoint[1]/height*(max_Y_degree-min_Y_degree)))
    
    #for debugging
    #print( "X,Y = ({},{})".format(midpoint[0],midpoint[1]))
    #print( "X,Y = ({},{})".format(X_degree,Y_degree))
    
    #sending the value to arduino
    send_to_arduino(X_degree,Y_degree)
    
def send_to_arduino(x,y):
    angle_value_list = [str(x),str(y)]
    send_string = ','.join(angle_value_list)
    send_string += "\n"
    
    # Send the string. Make sure you encode it before you send it to the Arduino.
    ser.write(send_string.encode('utf-8'))
    
    #for debugging
    #receive serial print from arduino for checking value that
    #rasp send
    #receive_string = ser.readline().decode('utf-8', 'replace').rstrip()
    #print(receive_string)
    



while(True):
    ret, frame= cap.read()
   
    frame = LIVE_CAM_ORB(frame)
    
    cv2.imshow("the actual frame", frame)
    
    #press enter to exit
    if cv2.waitKey(1) == 13:
        break
    time.sleep(0.25)


cap.release()
cv2.destroyAllWindows()
