"""
Example for PC in right corner

Setup right corner:
    Camera 1: Regular camera
    Camera 2: SH sensor
"""

import cv2
from skimage.filters import threshold_triangle

from camera.ueye_camera import uEyeCamera
from pyueye import ueye

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


#def find_centroid(img):
#    img = np.array(img)
#    if np.max(img) > 2:
#        img = img/255
#    rows, cols = np.indices(img.shape)
#    total_intensity = np.sum(img)
#    y_0 = np.sum(rows*img)/total_intensity
#    x_0 = np.sum(cols*img)/total_intensity
#    return int(x_0), int(y_0)


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


def grabframes(nframes, cameraIndex=1):
    with uEyeCamera(device_id=cameraIndex) as cam:
        cam.set_colormode(ueye.IS_CM_MONO8)
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

def eval_act(act):
    with OkoDM(dmtype=1) as dm:
        dm.setActuators(act)
        time.sleep(0.05)
    img=grabframes(3)
    return img[-1]

def generate_fx_initial(n_act=19):
    print("Generating initial values (may be slow)!")
    xlist = [np.random.rand(n_act)*2.0-1.0 for i in range(n_act+1)]
    flist = [psf_radius(eval_act(x)) for x in xlist]
    xlist_sorted = [x for _, x in sorted(zip(flist, xlist))]
    flist_sorted = sorted(flist)
    xlist_sorted = np.array(xlist_sorted)
    flist_sorted = np.array(flist_sorted)
    print(xlist_sorted.shape)
    print(flist_sorted.shape)
    np.savetxt(f"C:\\AO-course-2024\\part_4\\initial_x.dat", xlist_sorted)
    np.savetxt(f"C:\\AO-course-2024\\part_4\\initial_f.dat", flist_sorted)

def simplex(xlist_sorted, flist_sorted, n_act=19):
    i = 0
    iter_max = 1000
    #xlist = [np.random.rand(n_act)*2-1 for i in range(n_act+1)]
    #print("xlist set up")
    #flist = [psf_radius(eval_act(x)) for x in xlist]
    #print("flist set up")
    #xlist_sorted = [x for _, x in sorted(zip(flist, xlist))]
    #flist_sorted = sorted(flist)    
    metriclist = []
    
    while i < iter_max:
        xlist_sorted = [x for _, x in sorted(zip(flist_sorted, xlist_sorted))]
        flist_sorted = sorted(flist_sorted)
        metriclist.append(flist_sorted[-1])
        centroid = find_n_dim_centroid(xlist_sorted[:-1])

        if i%10 == 0:
           print(f"Iteration {i}, best PSF {psf_radius(eval_act(xlist_sorted[0]))}")
           img=eval_act(xlist_sorted[0])
           print(f"MAX pixel value: {np.max(img)}" )
           imsave(f"C:\\AO-course-2024\\part_4\\frames\\frame_{i+1}.png", img)
           np.savetxt("C:\\AO-course-2024\\part_4\\last_x.dat", xlist_sorted)
           np.savetxt("C:\\AO-course-2024\\part_4\\last_f.dat", flist_sorted)
           np.savetxt("C:\\AO-course-2024\\part_4\\metric.dat", metriclist)
        
        i += 1
        
        # Reflection
        alpha = 1
        x_reflected = centroid + alpha*(centroid-xlist_sorted[-1])
        x_reflected = np.clip(x_reflected, -1, 1)
        f_x_reflected = psf_radius(eval_act(x_reflected))
        if flist_sorted[0] <= f_x_reflected < flist_sorted[-2] :
            print(f"\tReflection: old {flist_sorted[-1]}, new {f_x_reflected}")
            xlist_sorted[-1] = x_reflected
            flist_sorted[-1] = f_x_reflected
            continue
        
        # Expansion
        if f_x_reflected < flist_sorted[0]:
            gamma = 2
            x_expanded = centroid + gamma*(x_reflected-centroid)
            x_expanded = np.clip(x_expanded, -1, 1)
            f_x_expanded = psf_radius(eval_act(x_expanded))
            if f_x_expanded < f_x_reflected:
                print(f"\tExpansion better than reflection: old {flist_sorted[-1]}, new {f_x_expanded}")
                xlist_sorted[-1] = x_expanded
                flist_sorted[-1] = f_x_expanded
                continue
            else:
                print(f"\tExpansion worse than reflection")
                xlist_sorted[-1] = x_reflected
                flist_sorted[-1] = f_x_reflected
                continue
        
        # Contraction
        rho = 0.7
        if f_x_reflected < flist_sorted[-1]:
            x_contracted = centroid + rho*(x_reflected-centroid)
            x_contracted = np.clip(x_contracted, -1, 1)
            f_x_contracted = psf_radius(eval_act(x_contracted))
            if f_x_contracted < f_x_reflected:
                print(f"\tContraction outside: old {flist_sorted[-1]}, new {f_x_contracted}")
                xlist_sorted[-1] = x_contracted
                flist_sorted[-1] = f_x_contracted
                continue
        else:
            x_contracted = centroid + rho*(xlist_sorted[-1]-centroid)
            x_contracted = np.clip(x_contracted, -1, 1)
            f_x_contracted = psf_radius(eval_act(x_contracted))
            if f_x_contracted < flist_sorted[-1]:
                print(f"\tContraction inside: old {flist_sorted[-1]}, new {f_x_contracted}")
                xlist_sorted[-1] = x_contracted
                flist_sorted[-1] = f_x_contracted
                continue
        
        # Shrink
        sigma = 0.7
        print("\tShrink!")
        for j in range(1, len(xlist_sorted)):
            xlist_sorted[j] = np.clip(xlist_sorted[0] + sigma*(xlist_sorted[j]-xlist_sorted[0]), -1, 1)
            flist_sorted[j] = psf_radius(eval_act(xlist_sorted[j]))
            

            
    
    return metriclist


if __name__ == "__main__":
    with OkoDM(dmtype=1) as dm:
        # generate_fx_initial()
#        x = np.loadtxt(f"C:\\AO-course-2024\\part_4\\initial_x.dat")
        
#        for ii in range(len(x)):
#            img = eval_act(x[ii])
#            print(psf_radius(img, ii))
##        print(x0, y0)
##        #img[img<10] = 0
##        print(psf_radius(img))
###        plt.clf()
###        plt.hist(img.flatten(), bins=256, range=(0, 256), density=True,)
###        plt.show()
#        plt.imshow(croppedimg)
        try:
            initial_x = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_x.dat")
            initial_f = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_f.dat")
        except:
            initial_x = np.loadtxt(f"C:\\AO-course-2024\\part_4\\initial_x.dat")
            initial_f = np.loadtxt(f"C:\\AO-course-2024\\part_4\\initial_f.dat")
#
#        initial_x = np.loadtxt(f"C:\\AO-course-2024\\part_4\\initial_x.dat")
#        initial_f = np.loadtxt(f"C:\\AO-course-2024\\part_4\\initial_f.dat")

        print(initial_f)
        metric = simplex(initial_x, initial_f)
        

#        metric = np.loadtxt(f"C:\\AO-course-2024\\part_4\\metric.dat")
#        plt.close('all')
#        plt.plot(range(len(metric)), metric)
#        plt.xlabel("Iterations")
#        plt.ylabel("PSF radius [px]")
#        plt.savefig("C:\\AO-course-2024\\part_4\\metric.png")
