import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)
#set the resolution
width =  640 #640 1280
height =  480 #480 720

#changing the resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)

MIN_MATCH_COUNT = 12

# Initiate ORB detector with max 1000 features and scaling 1.2
orb = cv2.ORB_create(1000, 1.2)

#input the reference image
input_image = cv2.imread('test1/raspberry pi.jpg')
input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
#search the keypoint of input image
kp1, des1 = orb.detectAndCompute(input_image,None)
#search the height and Width of the input image to make the boundary
h,w = input_image.shape
pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

# create BFMatcher object
bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)


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
        M, mask= cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        #(for debugging)
        #matchesMask = mask.ravel().tolist() 
        dst = cv2.perspectiveTransform(pts,M)
        (tl, tr, br, bl) = dst
        midpoint = (tl+bl+br+tr)/4
        live_cam = cv2.polylines(live_cam,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        live_cam = cv2.circle(live_cam,(int(midpoint[0][0]),int(midpoint[0][1])),5,255,-1)
        
        #for img3 (debugging with black and white)
        #img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
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

while(True):
    ret, frame= cap.read()
   
    live_cam = LIVE_CAM_ORB(frame)
    
    cv2.imshow("the actual frame", live_cam)
    
    #press enter to exit
    if cv2.waitKey(1) == 13:
        break
    time.sleep(0.05)


cap.release()
cv2.destroyAllWindows()