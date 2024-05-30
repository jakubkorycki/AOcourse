import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
from skimage.filters import threshold_otsu
from pyueye import ueye
from camera.ueye_camera import uEyeCamera
from dm.okotech.dm import OkoDM
import time
import os
from zernike import RZern

# Set the path for DLL loading
os.environ['PATH'] = "C:\\AO-course-2024\\dm\\okotech\\okodm_sdk\\python" + os.pathsep + os.environ['PATH']

os.chdir('./')

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

def reference_img(dm, sh_camera):
    optimized_act = np.loadtxt(f".\\part_4\\last_x.dat")[0]
    dm.setActuators(optimized_act)
    time.sleep(0.2)
    img = sh_camera.grab_frames(4)[-1]
    return img

def plot(img):
    plt.clf()
    plt.imshow(img, aspect='auto', interpolation='bicubic')

def plot_displacement(reference_dots, random_dots, indices):
    plt.scatter(reference_dots[:, 0], reference_dots[:, 1], 5, c='red')
    plt.scatter(random_dots[:, 0], random_dots[:, 1], 5, c='green')
    if len(reference_dots) > len(random_dots):
        for i, dot in enumerate(random_dots):
            plt.plot([reference_dots[indices[i], 0], random_dots[i, 0]], [reference_dots[indices[i], 1], random_dots[i, 1]], '-', c='black')
    else:
        for i, dot in enumerate(reference_dots):
            plt.plot([reference_dots[indices[i], 0], random_dots[i, 0]], [reference_dots[indices[i], 1], random_dots[i, 1]], '-', c='black')

def NearestNeighbor(reference_dots, random_dots):
    if len(reference_dots) > len(random_dots):
        tree = cKDTree(reference_dots)
        _, indices = tree.query(random_dots)
    else:
        tree = cKDTree(random_dots)
        _, indices = tree.query(reference_dots)
    return indices

def filter_points_by_neighbors(points, min_neighbours=4, neighbour_distance=65):
    tree = cKDTree(points)
    neighbours_count = np.array([len(tree.query_ball_point(point, neighbour_distance)) for point in points])
    filtered_points = points[neighbours_count >= min_neighbours]
    return filtered_points

def calculate_displacement(reference_dots, random_dots, indices):
    if len(reference_dots) > len(random_dots):
        displacement = reference_dots[indices] - random_dots
    else:
        displacement = reference_dots - random_dots[indices]
    return displacement

if __name__ == "__main__":
    plt.close('all')
    with OkoDM(dmtype=1) as dm:
        sh_camera = Camera(camera_index=2, exposure=3)
        normal_camera = Camera(camera_index=1, exposure=1)
        try:
            reference_img_normal = reference_img(dm, normal_camera)
            reference_img = reference_img(dm, sh_camera)
            reference_dots = detect_blobs(reference_img, show=True)
            print(len(reference_dots))
            dm.setActuators(np.random.rand(19) * 2 - 1)
            time.sleep(0.2)
            random_img = sh_camera.grab_frames(4)[-1]
            random_dots = detect_blobs(random_img, show=True)
            reference_dots = filter_points_by_neighbors(reference_dots)
            random_dots = filter_points_by_neighbors(random_dots)
        finally:
            pass

#%%


    plt.clf()
    indices = NearestNeighbor(reference_dots, random_dots)
    plot_displacement(reference_dots, random_dots, indices)
#%%
    zern = RZern(4)
    displacements = calculate_displacement(reference_dots, random_dots, indices)

    reference_dots_normalized = (reference_dots - np.mean(reference_dots, axis=0)) / np.ptp(reference_dots, axis=0)
    displacements_normalized = displacements / np.ptp(displacements, axis=0)
    
    L, K = 500, 500
    ddx = np.linspace(-1.0, 1.0, K)
    ddy = np.linspace(-1.0, 1.0, L)
    xv, yv = np.meshgrid(ddx, ddy)
    zern.make_cart_grid(xv, yv)
    
    points = reference_dots_normalized
    grid_points = np.c_[xv.flatten(), yv.flatten()]
    displacement_x_grid = griddata(points, displacements_normalized[:, 0], grid_points, method='cubic').reshape(xv.shape)
    displacement_y_grid = griddata(points, displacements_normalized[:, 1], grid_points, method='cubic').reshape(xv.shape)

    coeffs_x = zern.fit_cart_grid(displacement_x_grid.flatten())[0]
    coeffs_y = zern.fit_cart_grid(displacement_y_grid.flatten())[0]
    
    reconstructed_wavefront_x = zern.eval_grid(coeffs_x, matrix=True)
    reconstructed_wavefront_y = zern.eval_grid(coeffs_y, matrix=True)

    reconstructed_wavefront = reconstructed_wavefront_x + reconstructed_wavefront_y

    plt.figure(figsize=(12,6))
    plt.subplot(1, 2, 1)
    plt.imshow(reconstructed_wavefront, origin='lower', extent=(-1, 1, -1, 1))
    plt.colorbar()
    plt.title('Reconstructed Wavefront')

    plt.subplot(1, 2, 2)
    plt.plot(range(1, zern.nk + 1), coeffs_x, marker='.', label='X Coeffs')
    plt.plot(range(1, zern.nk + 1), coeffs_y, marker='.', label='Y Coeffs')
    plt.legend()
    plt.title('Zernike Coefficients')

    plt.show()
