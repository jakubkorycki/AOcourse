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
import copy
import time

import os
os.environ['PATH'] = "C:\\AO-course-2024\\dm\okotech\\okodm_sdk\\python" + os.pathsep + os.environ['PATH']

#os.add_dll_directory("C:\\AO-course-2023\\dm\okotech\\okodm_sdk")


SH_Sensor_Index = 2
Camera_Index = 1

def grabframes(nframes, cameraIndex=0):
    with uEyeCamera(device_id=1) as cam:
        cam.set_colormode(ueye.IS_CM_MONO8)#IS_CM_MONO8)
        w=1280
        h=1024
        #cam.set_aoi(0,0, w, h)
        
        cam.alloc(buffer_count=10)
        cam.set_exposure(0.3)
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


def centroid(img):
    '''
    # x_0 = range(np.shape(img)[0])
    print(np.shape(img))
    xcoords = range(np.shape(img)[0])
    ycoords = range(np.shape(img)[1])
    print(xcoords, ycoords)
    ycoords, xcoords  = np.indices(img.shape)
    #x_0 = np.sum([i*img[i,j] for i,j in zip(xcoords, ycoords)])/np.sum(img)
    x_0 = np.sum(xcoords*img)/np.sum(img)
    '''
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

if __name__ == "__main__":
    from dm.okotech.dm import OkoDM
    print("started")
    # Use "with" blocks so the hardware doesn't get locked when you press ctrl-c    
    with OkoDM(dmtype=1) as dm:
        # set actuators to 0
        act = np.zeros([len(dm)])
        dm.setActuators(act)
        img=grabframes(5)
        imsave(f"C:\\AO-course-2024\\part_3\\frames\\frame_0.png", img[-1])
        np.savetxt(f"C:\\AO-course-2024\\part_3\\raw\\frame_0.dat", img[-1])
        max_val = np.max(img[-1])
        
        # Run random walk loop
        if False:
            act=np.zeros([len(dm)])
            metriclist = []
            for i in range(500):
                # Pick random actuator, including tip-tilt for now
                rand_act_id = np.random.randint(0, len(dm))
                act_new = copy.copy(act)
                if abs(act_new[rand_act_id]) < 1.0:
                    act_new[rand_act_id] += 0.2 * np.random.randint(-1, 2)
                    
                dm.setActuators(act_new)
                time.sleep(0.1)
                img=grabframes(5)
                new_max = width(img[-1])
                
                if new_max < max_val:
                    act = act_new
                    metriclist.append(new_max)
                    max_val =  width(img[-1])
                    print("\nIMPROVED")
                    print(act)
                    
                    imsave(f"C:\\AO-course-2024\\part_3\\frames\\frame_{i}.png", img[-1])
                    np.savetxt(f"C:\\AO-course-2024\\part_3\\raw\\frame_{i}.dat", img[-1])
                    
                    if new_max >= 255:
                        print('Camera is saturated')
                        np.savetxt(f"C:\\AO-course-2024\\part_3\\final_actuator_values.dat", act)
                        break
                    print(i)
            # Save last, most corrected actuator voltages
            np.savetxt(f"C:\\AO-course-2024\\part_3\\final_actuator_values.dat", act)
            plt.figure()
            plt.plot(metriclist)
            plt.title("Metric vs number of iterations")
            plt.xlabel("Steps")
            plt.ylabel("max(I(x,y))")
            plt.savefig(f"C:\\AO-course-2024\\part_3\\metric_graph.png")
        # Set random initial values (including tip-tilt)
        if True:
            for i in range(20):
                act = np.random.uniform(-1,1,size=len(dm))
                print(f"Generating random initial position {i+1}")
                dm.setActuators(act)
                img=grabframes(5, 1)
                imsave(f"C:\\AO-course-2024\\part_3\\random_initial\\frames\\random_{i}.png", img[-1])
#                np.savetxt(f"C:\\AO-course-2024\\part_3\\random_initial\\raw\\random_{i}.dat", img[-1])
#                np.savetxt(f"C:\\AO-course-2024\\part_3\\random_initial\\voltages\\random_{i}_actuator_values.dat", act)
                
