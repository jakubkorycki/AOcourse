import cv2
import numpy as np
from scipy.spatial import cKDTree, distance_matrix
from scipy.interpolate import griddata
from sklearn.preprocessing import normalize
import matplotlib.pyplot as plt
from skimage.filters import threshold_otsu, threshold_triangle
from pyueye import ueye
from camera.ueye_camera import uEyeCamera
from dm.okotech.dm import OkoDM
import time
import os

from pathlib import Path
#os.chdir(Path(__file__).parent)
#from zern.zern import *

# Set the path for DLL loading
#os.environ['PATH'] = "C:\\AO-course-2024\\dm\\okotech\\okodm_sdk\\python" + os.pathsep + os.environ['PATH']

class Camera:
    def __init__(self, camera_index, exposure):
        self.camera_index = camera_index
        self.exposure = exposure

    def grab_frames(self, nframes):
        with uEyeCamera(device_id=self.camera_index) as cam:
            cam.set_colormode(ueye.IS_CM_MONO8)
            cam.alloc(buffer_count=10)
            cam.set_exposure(self.exposure)
            cam.capture_video(True)
            imgs = np.zeros((nframes, 1024, 1280), dtype=np.uint8)
            acquired = 0
            while acquired < nframes:
                frame = cam.grab_frame()
                if frame is not None:
                    imgs[acquired] = frame
                    acquired += 1
            cam.stop_video()
        return imgs

class DeformableMirror:
    def __init__(self, dm_type):
        self.dm_type = dm_type

    def setActuators(self, act):
        with OkoDM(dmtype=self.dm_type) as dm:
            dm.setActuators(act)
            time.sleep(0.1)

def detect_blobs(img, show=False):
#    thresh = threshold_triangle(img)
    thresh = threshold_otsu(img)
    _, binaryimg = cv2.threshold(img, thresh, 255, cv2.THRESH_TOZERO)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.blobColor = 255
    params.minArea = 25
    params.maxArea = 1000
    params.filterByCircularity = True
    params.minCircularity = 0.1
    params.filterByConvexity = True
    params.minConvexity = 0.5
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(binaryimg)
    keypoints = merge_close_keypoints(keypoints)
    if show:
        im_with_keypoints = cv2.drawKeypoints(binaryimg, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        plt.imshow(im_with_keypoints)
    return np.array([kp.pt for kp in keypoints])

def merge_close_keypoints(keypoints, min_distance=20):
    merged_keypoints = []
    used = [False] * len(keypoints)
    for i, kp1 in enumerate(keypoints):
        if used[i]:
            continue
        close_keypoints = [kp1]
        for j, kp2 in enumerate(keypoints):
            if i != j and not used[j]:
                dist = np.linalg.norm(np.array(kp1.pt) - np.array(kp2.pt))
                if dist < min_distance:
                    close_keypoints.append(kp2)
                    used[j] = True
        x = np.mean([kp.pt[0] for kp in close_keypoints])
        y = np.mean([kp.pt[1] for kp in close_keypoints])
        size = np.mean([kp.size for kp in close_keypoints])
        merged_keypoints.append(cv2.KeyPoint(x, y, size))
    return merged_keypoints

def reference_image(dm, camera):
    optimized_act = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_x.dat")[0]
    dm.setActuators(optimized_act)
    time.sleep(0.2)
    img = camera.grab_frames(4)[-1]
    return img

def plot(img):
    plt.clf()
    plt.imshow(img, aspect='auto', interpolation='bicubic')

def plot_displacement(reference_dots, random_dots):
    plt.scatter(reference_dots[:, 0], reference_dots[:, 1], 5, c='red')
    plt.scatter(random_dots[:, 0], random_dots[:, 1], 5, c='green')
    for i, dot in enumerate(random_dots):
        plt.plot([reference_dots[i, 0], random_dots[i, 0]], [reference_dots[i, 1], random_dots[i, 1]], '-', c='black')
   
def NearestNeighbor(reference_dots, random_dots):
    if len(reference_dots) > len(random_dots):
        tree = cKDTree(reference_dots)
        _, indices = tree.query(random_dots)
        
        return reference_dots[indices], random_dots
    else:
        tree = cKDTree(random_dots)
        _, indices = tree.query(reference_dots)
        
        return reference_dots, random_dots[indices]

def filter_points_by_neighbors(points, min_neighbours=4, neighbour_distance=65):
    tree = cKDTree(points)
    neighbours_count = np.array([len(tree.query_ball_point(point, neighbour_distance)) for point in points])
    filtered_points = points[neighbours_count >= min_neighbours]
    return filtered_points

def normalize_dots_to_unit_circle(dots, padding=0.1, center =None, scale=None,):
    if center is None:
        center = np.mean(dots, axis=0)
    dots_centered = dots - center
    if scale is None:
        scale = (1-padding)/np.max(np.sqrt(np.sum(dots_centered**2, axis=1)))
    dots_scaled = dots_centered*scale
    return dots_scaled, center, scale


if __name__ == "__main__":
    plt.close('all')
    pixel_size = 5.2e-6
    focal_length=6e-3
    with OkoDM(dmtype=1) as dm:
        sh_camera = Camera(camera_index=2, exposure=1)
        normal_camera = Camera(camera_index=1, exposure=0.5)
        try:
            reference_img = reference_image(dm, normal_camera)
            reference_SH_img = reference_image(dm, sh_camera)
            reference_dots = detect_blobs(reference_SH_img, show=False)
            optimized_act = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_x.dat")[0]
#            dm.setActuators(optimized_act)
#            randact = np.random.rand(19) *1.6-0.8
            optimized_act[1:5] = 1
            optimized_act[5:17] = -1
            dm.setActuators(randact)
            time.sleep(0.2)
            random_img = normal_camera.grab_frames(4)[-1]
            random_SH_img = sh_camera.grab_frames(4)[-1]
            random_dots = detect_blobs(random_SH_img, show=False)
            reference_dots = filter_points_by_neighbors(reference_dots)
            random_dots = filter_points_by_neighbors(random_dots)
            print(np.max(reference_img), np.max(reference_SH_img), np.max(random_img), np.max(random_SH_img))
        finally:
            pass
        
        
#%%

    plt.clf()
    reference_dots, random_dots = NearestNeighbor(reference_dots, random_dots)
    reference_dots_normalized, center, scale = normalize_dots_to_unit_circle(reference_dots)
    random_dots_normalized, _, _ = normalize_dots_to_unit_circle(random_dots, center=center, scale=scale)
    print(len(reference_dots), len(random_dots))
    plot_displacement(reference_dots_normalized, random_dots_normalized)
    
#%%
    
    displacements_normalized = reference_dots_normalized - random_dots_normalized
    ''' Normalization to unit circle happens in cell above!'''
#    reference_dots_normalized = (reference_dots - np.mean(reference_dots, axis=0)) / np.ptp(reference_dots, axis=0)
#    displacements_normalized = displacements / np.ptp(reference_dots, axis=0)
#    displacements_normalized = displacements_normalized.flatten()
#    reference_dots_normalized = reference_dots
    displacements_normalized = displacements_normalized.flatten()
    
    a, b = 2, 15
    zernike_gradients = np.zeros((b-a, len(reference_dots_normalized)*2))
    print(zernike_gradients.shape)
    
    for i in range(a, b):
        j=0
        n,m = noll2nm(i)
        
        for k in range(len(reference_dots_normalized)):
            x = reference_dots_normalized[k, 0]
            y = reference_dots_normalized[k, 1]
            zernike_gradients[i-2, j] =  (zernike_cartesian(n, m, x+1e-5, y) - zernike_cartesian(n, m, x, y))/1e-5
            zernike_gradients[i-2, j+1] =  (zernike_cartesian(n, m, x, y+1e-5) - zernike_cartesian(n, m, x, y))/1e-5
            j +=2
        
    B_inv = np.linalg.pinv(zernike_gradients.T)
    coefficient =  B_inv @ -displacements_normalized 
    
    N = 500
    x = np.linspace(-1, 1, N)
    y = np.linspace(-1, 1, N)
    X, Y = np.meshgrid(x, y)
    phi = np.zeros((N, N))
    
    for l in range(a,b):
        n,m = noll2nm(l)
        phi += coefficient[l-2]*zernike_cartesian(n, m, X, Y)
        
    
    plt.figure()
    plt.imshow(phi, cmap='viridis', origin='lower')
    plt.colorbar()
    plt.scatter(reference_dots_normalized[:, 0]*250+250, reference_dots_normalized[:, 1]*250+250, s=2, c='red')
    plt.scatter(random_dots_normalized[:, 0]*250+250, random_dots_normalized[:, 1]*250+250, s=2, c='cyan')

    plt.show()
