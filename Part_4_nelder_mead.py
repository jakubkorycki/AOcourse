"""
Example for PC in right corner

Setup right corner:
    Camera 1: Regular camera
    Camera 2: SH sensor
"""

from camera.ueye_camera import uEyeCamera
from pyueye import ueye

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imsave
from dm.okotech.dm import OkoDM

import os
import time
import copy
os.environ['PATH'] = "C:\\AO-course-2024\\dm\okotech\\okodm_sdk\\python" + os.pathsep + os.environ['PATH']

#os.add_dll_directory("C:\\AO-course-2023\\dm\okotech\\okodm_sdk")


SH_Sensor_Index = 2
Camera_Index = 1


def centroid(img):
    x_0 = np.uint64(0)
    y_0 = np.uint64(0)
    total_intensity = np.uint64(np.sum(img))
    for i in range(np.shape(img)[0]):
        for j in range(np.shape(img)[1]):
            intensity = np.uint64(img[i,j])
            #total_intensity += intensity
            x_0 += i*intensity
            y_0 += j*intensity
    
    x_0 /= total_intensity
    y_0 /= total_intensity
    return x_0, y_0

def width(img):
    # img[img>10] = 0
    x_0, y_0 = centroid(img)
    numerator = np.uint64(0)
    total_intensity = np.uint64(0)
    for i in range(np.shape(img)[0]):
        for j in range(np.shape(img)[1]):
            if img[i,j] > 10:
                total_intensity += img[i,j]
                numerator += np.uint64(img[i,j]*((i-x_0)**2+(j-y_0)**2))
    return np.sqrt(numerator/total_intensity)

def optimize(img):
    m = np.size(img)
    points = np.zeros(m+1)
    z = np.random.randn(m+1, m)
    z= img
    
    data = []
    feval = 0
    sum_f = 0
    sum_fz = np.zeros(m)
    
    for i in range(m+1):
        points[i] = np.var(z[i])
        feval += 1
        sum_fz += points[i]*z[i]
        sum_f += points[i]
    
    centroid = sum_fz/sum_f
    
    while feval < 1000:
        z[np.argmax(points)] = centroid
        points[np.argmax(points)] = np.var(centroid)
        feval += 1
        sum_f = 0
        sum_fz = np.zeros(m)
        for i in range(m+1):
            sum_fz += points[i]*z[i]
            sum_f += points[i]
            centroid = sum_fz/sum_f

        
        


def grabframes(nframes, cameraIndex=0):
    with uEyeCamera(device_id=1) as cam:
        cam.set_colormode(ueye.IS_CM_MONO8)#IS_CM_MONO8)
        w=1280
        h=1024
        #cam.set_aoi(0,0, w, h)
        
        cam.alloc(buffer_count=10)
        cam.set_exposure(6)
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

img = grabframes(5)[-1]
#print(centroid(grabframes(5)[-1]))
#print(width(img))



def eval_act(act):
    with OkoDM(dmtype=1) as dm:
        dm.setActuators(act)
        time.sleep(0.1)
        img=grabframes(5)
    return img[-1]

def simplex(n_act=19, step=1.0):
    x0 = np.random.rand(19)*2-1
    x1 = np.random.rand(19)*2-1
    x2 = np.random.rand(19)*2-1
    #print(x0)
    
    xs = np.array([x0, x1, x2])
    #print(xs)
    #fs = [width(eval_act(x) for x in xs]
    
    
    
    # result = np.array([[x0, f0], [x1, f1], [x2, f2]])
    i = 0
    iter_max = 10
    isw = np.zeros(4)
    while i < iter_max:

        step = 1.2 if i < 3 else 0.7
        xs = np.clip(xs, -1, 1)
        fs = [width(eval_act(x)) for x in xs]
        worst = xs[np.argmax(fs)]
        rest = np.delete(xs, np.argmax(fs), axis=0)
        xc = (rest[0]+rest[1])/2
        xnew = xc + step*(xc-worst)
        xnew = np.clip(xnew, -1, 1)
        with OkoDM(dmtype=1) as dm:
            dm.setActuators(xnew)
        img=grabframes(5)
        imsave(f"C:\\AO-course-2024\\consecutive_frames\\frame_{i+1}.png", img[-1])
        xs = np.append(rest, [xnew], axis=0)
        np.append(isw, width(eval_act(xnew)))
        print(width(eval_act(xnew)))
        print(f"Iteration {i}")
        i += 1
        
simplex(step=0.5)
#if __name__ == "__main__":
#
#    print("started")
#    # Use "with" blocks so the hardware doesn't get locked when you press ctrl-c    
#    with OkoDM(dmtype=1) as dm:
#        
#        # set actuators to 0
#        act = np.zeros([len(dm)])
#        dm.setActuators(act)
#        img=grabframes(5)
#        imsave(f"C:\\AO-course-2024\\consecutive_frames\\frame_0.png", img[-1])
#        np.savetxt(f"C:\\AO-course-2024\\consecutive_frames\\frame_0.dat", img[-1])
#        # Run loop
#        if True:
#            act=np.zeros([len(dm)])
#            for i in range(len(dm)):
#                act=np.zeros([len(dm)])
#                act[i] = 0.8
#                dm.setActuators(act)
#                time.sleep(0.1)
#                print(act)
#                plt.figure()
#                img=grabframes(5)
#                imsave(f"C:\\AO-course-2024\\consecutive_frames\\frame_{i+1}.png", img[-1])
#                np.savetxt(f"C:\\AO-course-2024\\consecutive_frames\\frame_{i+1}.dat", img[-1])
#                # plt.colorbar()
#                
#
#        act=np.zeros([len(dm)])
#        dm.setActuators(act)
#
#                
#        if False:
#            print(f"Deformable mirror with {len(dm)} actuators")
#            #dm.setActuators(np.random.uniform(-1,1,size=len(dm)))
#            
#            plt.figure()
#            img=grabframes(5, 1)
#            plt.imshow(img[-1])
#            plt.colorbar()
#            
#            plt.figure()
#            img=grabframes(5, 2)
#            plt.imshow(img[-1])
#            plt.colorbar()
#     