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
import time
if __name__ == "__main__":
    from dm.okotech.dm import OkoDM
    print("started")
    # Use "with" blocks so the hardware doesn't get locked when you press ctrl-c    
    with OkoDM(dmtype=1) as dm:
        
        # set actuators to 0
        act = np.zeros([len(dm)])
        dm.setActuators(act)
        img=grabframes(5)
        imsave(f"C:\\AO-course-2024\\consecutive_frames\\frame_0.png", img[-1])
        np.savetxt(f"C:\\AO-course-2024\\consecutive_frames\\frame_0.dat", img[-1])
        # Run loop
        if True:
            act=np.zeros([len(dm)])
            for i in range(len(dm)):
                act=np.zeros([len(dm)])
                act[i] = 0.8
                dm.setActuators(act)
                time.sleep(0.1)
                print(act)
                plt.figure()
                img=grabframes(5)
                imsave(f"C:\\AO-course-2024\\consecutive_frames\\frame_{i+1}.png", img[-1])
                np.savetxt(f"C:\\AO-course-2024\\consecutive_frames\\frame_{i+1}.dat", img[-1])
                # plt.colorbar()
                

        act=np.zeros([len(dm)])
        dm.setActuators(act)

                
        if False:
            print(f"Deformable mirror with {len(dm)} actuators")
            #dm.setActuators(np.random.uniform(-1,1,size=len(dm)))
            
            plt.figure()
            img=grabframes(5, 1)
            plt.imshow(img[-1])
            plt.colorbar()
            
            plt.figure()
            img=grabframes(5, 2)
            plt.imshow(img[-1])
            plt.colorbar()
            