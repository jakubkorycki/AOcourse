"""
Example for PC in right corner

Setup right corner:
    Camera 1: Regular camera
    Camera 2: SH sensor
"""

import cv2
from skimage.filters import threshold_triangle
from sklearn.linear_model import LinearRegression

from camera.ueye_camera import uEyeCamera
from pyueye import ueye

from sklearn.neighbors import NearestNeighbors
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imsave
#%matplotlib qt
from dm.okotech.dm import OkoDM

import os
import time
import copy
os.environ['PATH'] = "C:\\AO-course-2024\\dm\okotech\\okodm_sdk\\python" + os.pathsep + os.environ['PATH']

#os.add_dll_directory("C:\\AO-course-2023\\dm\okotech\\okodm_sdk")


SH_Sensor_Index = 2
Camera_Index = 1


def find_centroid(img):
    img = np.array(img)
    if np.max(img) > 2:
        img = img/255
    rows, cols = np.indices(img.shape)
    total_intensity = np.sum(img)
    y_0 = np.sum(rows*img)/total_intensity
    x_0 = np.sum(cols*img)/total_intensity
    return int(x_0), int(y_0)


def threshold_image(img):
    thresh = threshold_triangle(img)
    thresh, threshimg = cv2.threshold(img, thresh, 255, cv2.THRESH_TOZERO)
    
    return threshimg
    

def find_centroid(img):
    img = np.array(img)
    M = cv2.moments(img)
    x0 = int(M["m10"]/M["m00"])
    y0 = int(M["m01"]/M["m00"])

    return x0, y0


def find_n_dim_centroid(vectors):
    n = len(vectors[0])
    centroid = [0]*n
    
    for vector in vectors:
        for i in range(n):
            centroid[i] += vector[i]
    
    centroid = [x/len(vectors) for x in centroid]
    return centroid

def psf_radius(img):
    img = np.array(img)

    threshimg = threshold_image(img)
    x_0, y_0 = find_centroid(threshimg)
    # Image cropping
    croppedimg = threshimg[y_0-100:y_0+100, x_0-100:x_0+100]
    x_0, y_0 = find_centroid(croppedimg)
#    croppedimg[:20, x_0] = 255
#    croppedimg[y_0, :20] = 255
    
    # PSF radius calculation
    rows, cols = np.indices(croppedimg.shape)
    croppedimg = croppedimg/255
    radius = np.sqrt(np.sum(croppedimg * ((cols - x_0) ** 2 + (rows - y_0) ** 2))/np.sum(croppedimg))
    
    return radius


def grabframes(nframes, cameraIndex=0):
    with uEyeCamera(device_id=cameraIndex) as cam:
        cam.set_colormode(ueye.IS_CM_MONO8)
        w=1280
        h=1024
        #cam.set_aoi(0,0, w, h)
        
        cam.alloc(buffer_count=10)
        # TODO: set exposure high for SH, low for camera
        cam.set_exposure(3)
        cam.capture_video(True)
    
        imgs = np.zeros((nframes,h,w),dtype=np.uint8)
        acquired=0
        # For some reason, the IDS cameras seem to be overexposed on the first frames (ignoring exposure time?). 
        # So best to discard some frames and then use the last one
        while acquired<nframes:
            frame = cam.grab_frame()
            if frame is not None:
                imgs[acquired]=frame
                acquired+=1
    
        cam.stop_video()
    
    return imgs

def eval_act(act):
    with OkoDM(dmtype=1) as dm:
        dm.setActuators(act)
        time.sleep(0.05)
    img=grabframes(3)
    return img[-1]

def detect_features(img):
    dot_positions = []
#    height, width = img.shape
#    img = cv2.resize(img, (width//4, height//4))
    
    # blurredimg = cv2.GaussianBlur(img, (8,8), 0)
    _, binaryimg = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY)
#    plt.imshow(binaryimg)
    
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
#    params.minThreshold = 0
#    params.maxThreshold = 255
    
    
    # Filter by Area.
    params.filterByArea = True
    params.blobColor = 255
    params.minArea = 20
    params.maxArea = 1000
    
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.01
    
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5
    
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(binaryimg)
    im_with_keypoints = cv2.drawKeypoints(binaryimg, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    plt.imshow(im_with_keypoints)
    dot_positions = [keypoint.pt for keypoint in keypoints]
    print(dot_positions)
    print(len(dot_positions))
    return dot_positions

def find_rotation_angle(dot_positions):
    # TODO: INSTEAD, maybe try cv2.cornerHarris corner detection!
    xcoords = np.array([pos[0] for pos in dot_positions]).reshape(-1, 1) # into col vector
    ycoords = np.array([pos[1] for pos in dot_positions])
    # This can be done simpler? Also not quite sure it works
    model = LinearRegression().fit(xcoords, ycoords)
    angle = np.arctan(model.coef_[0])
    
#    angle = 0.5*np.pi-angle
    return angle
    
    
def rotate_img(img, angle):
    h, w = img.shape
    center = find_centroid(img)
    M = cv2.getRotationMatrix2D(center, np.degrees(angle), 1.0)
    newimg = cv2.warpAffine(img, M, (w,h))
    return newimg




if __name__ == "__main__":
    with OkoDM(dmtype=1) as dm:
        img = grabframes(3, 2)[-1]
        dot_positions = detect_features(img)
        plt.show()
#        rotation_angle = find_rotation_angle(dot_positions)
#        print(f"Angle: {np.degrees(rotation_angle)} deg")
#        # Note the fudge factor of 5
#        img = rotate_img(img, np.degrees(-rotation_angle))
#        plt.clf()
#        plt.imshow(img)
#        point_mask = np.zeros_like(img)
#        for x, y in keypoints:
#            point_mask[x-10:x+10, y-10:y+10] = 255
##        point_mask[keypoints] = 255
#        plt.imshow(point_mask)
