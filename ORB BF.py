import numpy as np
import cv2
from matplotlib import pyplot as plt

MIN_MATCH_COUNT = 10

img1 = cv2.imread('test1/box.png')          # queryImage
img2 = cv2.imread('test1/box_in_scene.png') # trainImage

# Initiate ORB detector
orb = cv2.ORB_create(1000, 1.2)

# find the keypoints and descriptors with ORB
kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

# Match descriptors.
matches = bf.knnMatch(des1,des2, k=2)

# store all the good matches as per Lowe's ratio test.
good = []
for m,n in matches:
    if m.distance < 0.7*n.distance:
        good.append(m)

if len(good)>MIN_MATCH_COUNT:
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    matchesMask = mask.ravel().tolist()
    h,w,d = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)
    box = np.array(dst, dtype="int")
    (tl, bl, br, tr) = box
    midpoint = (tl[0]+bl[0]+br[0]+tr[0])/4
    #midpoint = np.rint(midpoint) //round the integer
    midpoint = midpoint.astype(int)
    print( "pst = ({})".format(pts) )
    print( "dst = ({})".format(dst) )
    print( "box = ({})".format(box) )
    print( "top left = ({})".format(tl) )
    print( "bottom left = ({})".format(bl) )
    print( "top right = ({})".format(tr) )
    print( "bottom right = ({})".format(br) )
    print( "midpoint 0 = ({})".format(midpoint[0]) )
    print( "midpoint 1 = ({})".format(midpoint[1]) )    

    img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
    img2 = cv2.circle(img2,(midpoint[0],midpoint[1]),5,255,-1)
else:
    print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
    matchesMask = None


draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)


img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

plt.imshow(img3, 'gray'),plt.show()
plt.imshow(img2, 'gray'),plt.show()
