import cv2
import numpy as np
from scipy.spatial import cKDTree
#from scipy.interpolate import griddata
#from sklearn.preprocessing import normalize
import matplotlib.pyplot as plt
from skimage.filters import threshold_otsu#, threshold_triangle
from pyueye import ueye
from camera.ueye_camera import uEyeCamera
from dm.okotech.dm import OkoDM
import time
import os

from pathlib import Path
os.chdir(Path(__file__).parent)
#from zernike import RZern
from zern.zern import noll2nm, zernike_cartesian

# Set the path for DLL loading
os.environ['PATH'] = "C:\\AO-course-2024\\dm\\okotech\\okodm_sdk\\python" + os.pathsep + os.environ['PATH']

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

def create_reference_image(dm, camera):
    optimized_act = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_x.dat")[0]
    dm.setActuators(optimized_act)
    time.sleep(0.2)
    img = camera.grab_frames(4)[-1]
    return img

def plot(img):
    plt.clf()
    plt.imshow(img, aspect='auto', interpolation='bicubic')

def plot_displacement(reference_grid, distorted_grid):
    plt.scatter(reference_grid[:, 0], reference_grid[:, 1], 5, c='red')
    plt.scatter(distorted_grid[:, 0], distorted_grid[:, 1], 5, c='green')
    for i, dot in enumerate(distorted_grid):
        plt.plot([reference_grid[i, 0], distorted_grid[i, 0]], [reference_grid[i, 1], distorted_grid[i, 1]], '-', c='black')
   
def NearestNeighbor(reference_grid, distorted_grid):
    if len(reference_grid) > len(distorted_grid):
        tree = cKDTree(reference_grid)
        _, indices = tree.query(distorted_grid)
        
        return reference_grid[indices], distorted_grid
    else:
        tree = cKDTree(distorted_grid)
        _, indices = tree.query(reference_grid)
        
        return reference_grid, distorted_grid[indices]

def filter_points_by_neighbors(points, min_neighbours=4, neighbour_distance=65):
    tree = cKDTree(points)
    neighbours_count = np.array([len(tree.query_ball_point(point, neighbour_distance)) for point in points])
    filtered_points = points[neighbours_count >= min_neighbours]
    return filtered_points

def create_reference_grid(dm, camera):
    reference_SH_image = create_reference_image(dm, camera)
    reference_grid = detect_blobs(reference_SH_image, show=False)
    reference_grid = filter_points_by_neighbors(reference_grid)
    np.savetxt("reference_grid.csv", reference_grid)
    return reference_grid
    
def normalize_reference_grid_to_unit_circle(reference_grid, padding=0.0, center =None, scale=None,):
    if center is None:
        center = np.mean(reference_grid, axis=0)
    reference_grid_centered = reference_grid - center
    if scale is None:
        scale = (1-padding)/np.max(np.sqrt(np.sum(reference_grid_centered**2, axis=1)))
    reference_grid_scaled = reference_grid_centered*scale
    return reference_grid_scaled, center, scale

def create_B_matrix(reference_grid_normalized, n_modes):
    lowest_mode = 2
    zernike_gradients = np.zeros((n_modes-lowest_mode, len(reference_grid)*2))
    for i in range(lowest_mode, n_modes):
        j=0
        n,m = noll2nm(i)
        for k in range(len(reference_grid)):
            x = reference_grid_normalized[k, 0]
            y = reference_grid_normalized[k, 1]
            zernike_gradients[i-2, j] =  (zernike_cartesian(n, m, x+1e-5, y) - zernike_cartesian(n, m, x, y))/1e-5
            zernike_gradients[i-2, j+1] =  (zernike_cartesian(n, m, x, y+1e-5) - zernike_cartesian(n, m, x, y))/1e-5
            j +=2
    B = np.linalg.pinv(zernike_gradients.T)
    np.savetxt(f"B_matrix.csv", B)
    return B


def normalize_grids(reference_grid, distorted_grid):
    reference_grid_normalized, center, scale = normalize_reference_grid_to_unit_circle(reference_grid)
    distorted_grid_normalized, _, _ = normalize_reference_grid_to_unit_circle(distorted_grid, center=center, scale=scale)
    return reference_grid_normalized, distorted_grid_normalized

def get_slopes(reference_grid_normalized, distorted_grid_normalized):
    displacements = (reference_grid_normalized - distorted_grid_normalized).flatten()
    return displacements

def get_current_grid(camera):
    img = camera.grab_frames(4)[-1]
    grid = detect_blobs(img, show=False)
    grid = filter_points_by_neighbors(grid)
    return grid


def reconstruct_wavefront(B, displacements, n_modes=10, gridsize=500):
    coefficients = B @ -displacements
    x = np.linspace(-1, 1, gridsize)
    y = np.linspace(-1, 1, gridsize)
    X, Y = np.meshgrid(x, y)
    wavefront = np.zeros((gridsize, gridsize))
    for l in range(2, n_modes):
        n,m = noll2nm(l)
        wavefront += coefficients[l-2]*zernike_cartesian(n, m, X, Y)
    return wavefront

def create_C_matrix(dm, camera, reference_grid, voltage_step=0.5):
    num_actuators = 19
    len_grid = len(reference_grid)
    C = np.zeros((len_grid * 2, num_actuators))

    for i in range(num_actuators):
        act = np.zeros(num_actuators)
        act[i] = voltage_step
        dm.setActuators(act)
        time.sleep(0.1)
        positive_image = camera.grab_frames(4)[-1]
        positive_grid = detect_blobs(positive_image, show=False)
        positive_grid = filter_points_by_neighbors(positive_grid)
        reference_grid, positive_grid = NearestNeighbor(reference_grid, positive_grid)

        act[i] = -voltage_step
        dm.setActuators(act)
        time.sleep(0.1)
        negative_image = camera.grab_frames(4)[-1]
        negative_grid = detect_blobs(negative_image, show=False)
        negative_grid = filter_points_by_neighbors(negative_grid)
        _, negative_grid = NearestNeighbor(reference_grid, negative_grid)

        _, positive_grid = normalize_grids(reference_grid, positive_grid)
        _, negative_grid = normalize_grids(reference_grid, negative_grid)
        slope_x = (positive_grid[:, 0] - negative_grid[:, 0]) / (2 * voltage_step)
        slope_y = (positive_grid[:, 1] - negative_grid[:, 1]) / (2 * voltage_step)
        
        C[:len_grid, i] = slope_x
        C[len_grid:, i] = slope_y
        print(f"C col {i}")

    np.savetxt("C_matrix.csv", C)
    return C

def plot_wavefront(wavefront, reference_grid_normalized, distorted_grid_normalized):
    plt.figure()
    plt.imshow(wavefront, cmap='viridis')
    plt.colorbar()
    plt.scatter((reference_grid_normalized[:, 0]+1)*grid_size//2, (reference_grid_normalized[:, 1]+1)*grid_size//2, s=2, c='red')
    plt.scatter((distorted_grid_normalized[:, 0]+1)*grid_size//2, (distorted_grid_normalized[:, 1]+1)*grid_size//2, s=2, c='black')
    #plt.show()
    
def calculate_desired_slope_pattern(zernike_coeffs, reference_grid, n_modes):
    lowest_mode = 2
    desired_slope_pattern = np.zeros(len(reference_grid) * 2)
    for l in range(lowest_mode, n_modes):
        n, m = noll2nm(l)
        for k in range(len(reference_grid)):
            x = reference_grid[k, 0]
            y = reference_grid[k, 1]
            desired_slope_pattern[2*k] += zernike_coeffs[l-2] * (zernike_cartesian(n, m, x+1e-5, y) - zernike_cartesian(n, m, x, y)) / 1e-5
            desired_slope_pattern[2*k+1] += zernike_coeffs[l-2] * (zernike_cartesian(n, m, x, y+1e-5) - zernike_cartesian(n, m, x, y)) / 1e-5
    return desired_slope_pattern

def calculate_desired_voltages(C, desired_slope_pattern):
    # The truncation below might be a mistake/bandaid fix
    return np.linalg.pinv(C[:len(desired_slope_pattern)]) @ desired_slope_pattern

def get_current_slopes(reference_grid, camera):
    current_image = camera.grab_frames(4)[-1]
    current_grid = detect_blobs(current_image, show=False)
    current_grid = filter_points_by_neighbors(current_grid)
    reference_grid, current_grid = NearestNeighbor(reference_grid, current_grid)
    reference_grid_normalized, current_grid_normalized = normalize_grids(reference_grid, current_grid)
    return get_slopes(reference_grid_normalized, current_grid_normalized)

def update_voltages(dm, current_voltages, voltage_correction):
    new_voltages = current_voltages + voltage_correction
    dm.setActuators(new_voltages)
    return new_voltages

def converge_to_zernike(dm, camera, reference_grid, C, zernike_coeffs, n_modes, max_iterations=100, tolerance=1e-6):
    desired_slope_pattern = calculate_desired_slope_pattern(zernike_coeffs, reference_grid, n_modes)
    print(desired_slope_pattern) # Checked for tip, tilt modes and it seems correct
    # desired_voltages = calculate_desired_voltages(C, desired_slope_pattern)
    current_voltages = np.zeros(19)
    dm.setActuators(current_voltages)
    time.sleep(0.1)

    for iteration in range(max_iterations):
        current_slopes = get_current_slopes(reference_grid, camera)
        current_grid = get_current_grid(camera)
        reference_grid, current_grid = NearestNeighbor(reference_grid, current_grid)
        print(f"slope difference: {desired_slope_pattern - current_slopes}")
        voltage_correction = calculate_desired_voltages(C, desired_slope_pattern - current_slopes)
        print(f"Voltage correction: {voltage_correction}")
        current_voltages = update_voltages(dm, current_voltages, voltage_correction)
        if np.linalg.norm(voltage_correction) < tolerance:
            print(f"Converged after {iteration+1} iterations.")
            break
    else:
        reference_grid_normalized, current_grid_normalized = normalize_grids(reference_grid, current_grid)
        print("Max iterations reached without convergence.")
    return current_voltages, current_grid_normalized
    
    


if __name__ == "__main__": 
    plt.close('all')
    with OkoDM(dmtype=1) as dm:
        sh_camera = Camera(camera_index=2, exposure=1)
        normal_camera = Camera(camera_index=1, exposure=0.5)
        
        if False:
            reference_grid = np.loadtxt("reference_grid.csv")
            print("Reference grid loaded from file")
        else:
            reference_grid = create_reference_grid(dm=dm, camera=sh_camera)
            print("Reference grid measured")
        
        reference_grid_normalized, _, _ = normalize_reference_grid_to_unit_circle(reference_grid)
        
        # Decide whether to load from file or generate B and C matrices
        if False:
            B_matrix = np.loadtxt("B_matrix.csv")
            print("B matrix loaded from file")
        else:
            n_modes_B = 100
            B_matrix = create_B_matrix(reference_grid_normalized, n_modes_B)
            print(f"B matrix (n={n_modes_B}) has been calculated and saved")
        
        '''Sometimes dimensions don't match due to dropout, requiring recalculation every time. \
        Maybe there's a way to prevent that?'''
        if True:
            C_matrix = np.loadtxt("C_matrix.csv")
            print("C matrix loaded from file")
        else:
            C_matrix = create_C_matrix(dm, sh_camera, reference_grid)
            print("Influence matrix C has been calculated and saved")
            
        
        '''Testing wavefront reconstruction for given actuator settings'''
        if False:
            # Creating distorted grid, e.g. for defocus-like (first 8 actuators)
            optimized_act = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_x.dat")[0]
            # rand_act = np.random.rand(19)*1.6 -0.8
            # rand_act[-2:] = optimized_act[-2:]
            rand_act = optimized_act
            rand_act[:8] = -0.8
            dm.setActuators(rand_act)
            time.sleep(0.1)
            distorted_grid = get_current_grid(sh_camera)
            
    
            # Matching grid and normalizing
            reference_grid, distorted_grid = NearestNeighbor(reference_grid, distorted_grid)
            reference_grid_normalized, distorted_grid_normalized = normalize_grids(reference_grid, distorted_grid)
            slopes = get_slopes(reference_grid_normalized, distorted_grid_normalized)
            grid_size = 500
            
            # Plot wavefront of rand_act
            wavefront = reconstruct_wavefront(B_matrix, slopes, n_modes=20, gridsize=grid_size)
            plot_wavefront(wavefront, reference_grid_normalized, distorted_grid_normalized)
            plt.show()

        
        '''Control mirror to achieve wf corresponding to desired zernike coeffs'''
        n_modes = 12
        zernike_coeffs = np.zeros(n_modes)
        zernike_coeffs[1] = 0.1
        voltages, current_grid_normalized = converge_to_zernike(dm, sh_camera, reference_grid, C_matrix, zernike_coeffs, n_modes)
        
        dm.setActuators(voltages)
        time.sleep(0.1)
        
        slopes = get_current_slopes(reference_grid, sh_camera)
        print(slopes)
        
        grid_size = 500
        wavefront = reconstruct_wavefront(B_matrix, slopes, n_modes=20, gridsize=grid_size)
        plot_wavefront(wavefront, reference_grid_normalized, current_grid_normalized)
        