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
from zernike import RZern
import sys
from pathlib import Path
os.chdir(Path(__file__).parent)
from zern.zern import *
from scipy.fftpack import fft2, fftshift, ifftshift, ifft2


# Set the path for DLL loading
#os.environ['PATH']= "C:\\AO-course-2024\\dm\\okotech\\okodm_sdk\\python" + os.pathsep + os.environ['PATH']
sys.path.append("C:\\AO-course-2024\\dm\\okotech\\okodm_sdk\\python")


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
    params.minCircularity = 0.5
    params.filterByConvexity = True
    params.minConvexity = 0.3
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

def compute_fft_image(image):
    fft_result = fft2(image)
    fft_result_shift = fftshift(fft_result)
    fft_magnitude = fft_result_shift.real
    fft_phase = fft_result_shift.imag
    return fft_magnitude, fft_phase

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
#            test = np.zeros(19)
#            test[5] = 1
            optimized_act = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_x.dat")[0]
#            optimized_act[0]=1
            dm.setActuators(optimized_act)
            dm.setActuators(np.random.rand(19) * 2 - 1)
            time.sleep(0.2)
            random_img = normal_camera.grab_frames(4)[-1]
            random_SH_img = sh_camera.grab_frames(4)[-1]
            random_dots = detect_blobs(random_SH_img, show=False)
#            random_dots = reference_dots
            reference_dots = filter_points_by_neighbors(reference_dots)
            random_dots = filter_points_by_neighbors(random_dots)
            print(np.max(reference_img), np.max(reference_SH_img), np.max(random_img), np.max(random_SH_img))
        finally:
            pass
        
        
#%%
plt.clf()
#plt.clear()
        
def coords_to_img(coords):
    s = int(np.max(coords)+20)
    image = np.zeros((s, s), dtype=np.uint8)
    for x,y in coords:
        image[int(y), int(x)] = 255
    return image

plt.clf()
reference_dots, random_dots = NearestNeighbor(reference_dots, random_dots)
print(len(reference_dots), len(random_dots))
plot_displacement(reference_dots, random_dots)
plt.show()
#%%
plt.clf()
refimg = coords_to_img(reference_dots)
fft_mag, fft_phase = compute_fft_image(refimg)

scale_factor = 0.6

s = fft_mag.shape[0]

#xmin = s*0.5*(1-scale_factor)
#xmax = s 
fft_mag = fft_mag[200:600, 200:600]
fft_phase = fft_phase[200:600, 200:600]


plt.imshow(np.sqrt(fft_mag**2+fft_phase**2))
plt.colorbar()
plt.show()

#%%
plt.clf()
fft_reversed = np.empty(fft_mag.shape, dtype=complex)

fft_reversed.real = fft_mag
fft_reversed.imag = fft_phase

fft_reversed_shift = ifftshift(fft_reversed)
fft_reversed = ifft2(fft_reversed_shift)

fft_reversed_mag = fft_reversed.real
fft_reversed_phase = fft_reversed.imag

plt.imshow(np.sqrt(fft_reversed_mag**2+fft_reversed_phase**2))
plt.show()


#%%

def calc_B_matrix(reference_dots, n_modes):
    

    displacements = reference_dots - random_dots
    
    reference_dots_normalized = (reference_dots - np.mean(reference_dots, axis=0)) / np.ptp(reference_dots, axis=0)
    displacements_normalized = displacements / np.ptp(reference_dots, axis=0)
    displacements_normalized = displacements_normalized.flatten()
    
#    a, b = 2, 15
    zernike_gradients = np.zeros((n_modes, len(reference_dots)*2))
    
    for i in range(2, 2+n_modes):
        j=0
        n,m = noll2nm(i)
        
        for k in range(len(reference_dots)):
            x = reference_dots_normalized[k, 0]
            y = reference_dots_normalized[k, 1]
            zernike_gradients[i-2, j] =  (zernike_cartesian(n, m, x+1e-5, y) - zernike_cartesian(n, m, x, y))/1e-5
            zernike_gradients[i-2, j+1] =  (zernike_cartesian(n, m, x, y+1e-5) - zernike_cartesian(n, m, x, y))/1e-5
            j +=2
        

B = calc_B_matrix(reference_dots, 13)
B = np.linalg.pinv(zernike_gradients.T)
coefficient = B @ displacements_normalized
    
    
x = np.linspace(-1, 1, 200)
y = np.linspace(-1, 1, 200)
X, Y = np.meshgrid(x, y)
phi = np.zeros((200, 200))

for l in range(a,b):
    n,m = noll2nm(l)
    phi += coefficient[l-2]*zernike_cartesian(n, m, X, Y)

plt.figure()
plt.imshow(phi)
plt.colorbar()



                         
        
        
    
    
    
    
    
#%%
#
#    displacements_normalized = (displacements  - np.min(displacements, axis=0)) / np.ptp(displacements, axis=0)
##    reference_dots_normalized = (reference_dots - np.mean(reference_dots, axis=0)) / np.ptp(reference_dots, axis=0)
##    displacements_normalized = displacements / np.ptp(displacements, axis=0)
#
#
#    
#    L, K = 200, 250
#    ddx = np.linspace(-1.0, 1.0, K)
#    ddy = np.linspace(-1.0, 1.0, L)
#    xv, yv = np.meshgrid(ddx, ddy)
#    zern.make_cart_grid(xv, yv)
#    
#    points = reference_dots_normalized
#    grid_points = np.c_[xv.flatten(), yv.flatten()]
#    displacement_x_grid = griddata(points, displacements_normalized[:, 0], grid_points, method='cubic').reshape(xv.shape)
#    displacement_y_grid = griddata(points, displacements_normalized[:, 1], grid_points, method='cubic').reshape(xv.shape)
#
#    coeffs_x = zern.fit_cart_grid(displacement_x_grid.flatten())[0]
#    coeffs_y = zern.fit_cart_grid(displacement_y_grid.flatten())[0]
#    
#    reconstructed_wavefront_x = zern.eval_grid(coeffs_x, matrix=True)
#    reconstructed_wavefront_y = zern.eval_grid(coeffs_y, matrix=True)
#
#    reconstructed_wavefront = reconstructed_wavefront_x + reconstructed_wavefront_y
#
#    plt.figure(figsize=(12,6))
#    plt.subplot(1, 2, 1)
#    plt.imshow(reconstructed_wavefront, origin='lower', extent=(-1, 1, -1, 1))
#    plt.colorbar()
#    plt.title('Reconstructed Wavefront')
#
#    plt.subplot(1, 2, 2)
#    plt.plot(range(1, zern.nk + 1), coeffs_x, marker='.', label='X Coeffs')
#    plt.plot(range(1, zern.nk + 1), coeffs_y, marker='.', label='Y Coeffs')
#    plt.legend()
#    plt.title('Zernike Coefficients')
#
#    plt.show()

#%%
