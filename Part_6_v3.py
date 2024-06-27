import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import griddata
#from scipy.interpolate import griddata
#from sklearn.preprocessing import normalize
import matplotlib.pyplot as plt
from skimage.filters import threshold_otsu, threshold_triangle
from pyueye import ueye
from camera.ueye_camera import uEyeCamera
from dm.okotech.dm import OkoDM
import time
import os
from skimage.feature import canny
from skimage.transform import hough_line, hough_line_peaks, radon


from pathlib import Path
os.chdir(Path(__file__).parent)
from zern.zern import noll2nm, zernike_cartesian

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

def detect_blobs(img, show=False, index=-1):
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
        plt.savefig(f"{int(index)}.png")
        plt.clf()
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

def filter_points_by_neighbors(points, min_neighbours=7, neighbour_distance=89):
    tree = cKDTree(points)
    neighbours_count = np.array([len(tree.query_ball_point(point, neighbour_distance)) for point in points])
    filtered_points = points[neighbours_count >= min_neighbours]
    return filtered_points

def create_reference_grid(dm, camera):
    reference_SH_image = create_reference_image(dm, camera)
    plt.imshow(reference_SH_image)
    reference_grid = detect_blobs(reference_SH_image, show=False)
    reference_grid = filter_points_by_neighbors(reference_grid)
    np.savetxt("reference_grid.csv", reference_grid)
    return reference_grid
    
def normalize_reference_grid_to_unit_circle(reference_grid, padding=0.1, center =None, scale=None,):
    if center is None:
        center = np.mean(reference_grid, axis=0)
    reference_grid_centered = reference_grid - center
    if scale is None:
        scale = (1-padding)/np.max(np.sqrt(np.sum(reference_grid_centered**2, axis=1)))
    reference_grid_scaled = reference_grid_centered*scale
    return reference_grid_scaled, center, scale

def create_B_matrix(reference_grid_normalized, n_modes):
    lowest_mode = 2
    zernike_gradients = np.zeros((n_modes, len(reference_grid)*2))
    print(zernike_gradients.shape)
    
    for i in range(lowest_mode, n_modes+lowest_mode):
        j=0
        n,m = noll2nm(i)
        for k in range(len(reference_grid)):
            x = reference_grid_normalized[k, 0]
            y = reference_grid_normalized[k, 1]
            zernike_gradients[i-lowest_mode, j] =  (zernike_cartesian(n, m, x+1e-5, y) - zernike_cartesian(n, m, x, y))/1e-5
            zernike_gradients[i-lowest_mode, j+1] =  (zernike_cartesian(n, m, x, y+1e-5) - zernike_cartesian(n, m, x, y))/1e-5
            j += 2
            
    B = np.linalg.pinv(zernike_gradients.T)
    print(B.shape)
    np.savetxt("B_matrix.csv", B)
    return B, zernike_gradients

def find_rotation_angle(grid):
    p1 = grid[32]
    p2 = grid[33]
    dx = p2[0]-p1[0]
    dy = p2[1]-p1[1]
    rotation_angle = np.arctan(dy/dx)
    return rotation_angle

#def interpolate_missing_spots(current_grid, reference_grid):
#    tree = cKDTree(current_grid)
#    
#    missing_indices = [i for i in range(len(reference_grid)) if not np.any(tree.query_ball_point(reference_grid[i], r=25))]
#    interpolated_points = griddata(current_grid, current_grid, reference_grid[missing_indices], method='cubic')
#    print(f'Interpolated {len(interpolated_points)} points.\n')
#    if len(interpolated_points) > 2:
#        np.savetxt('interpolated_grid.dat', np.vstack([current_grid, interpolated_points]))
#    
##    interpolated_grid = NearestNeighbor(reference_grid, np.vstack([current_grid, interpolated_points]))
#    interpolated_grid = np.vstack([current_grid, interpolated_points])
#    return interpolated_grid

def find_centroid(img):
    img = np.array(img)
    M = cv2.moments(img)
    x0 = int(M["m10"] / M["m00"])
    y0 = int(M["m01"] / M["m00"])
    return x0, y0

def rotate_img(img, angle):
    h, w = img.shape
    center = find_centroid(img)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    newimg = cv2.warpAffine(img, M, (w, h))
    return newimg


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
    lowest_mode = 2
    coefficients = B @ displacements
    x = np.linspace(-1, 1, gridsize)
    y = np.linspace(-1, 1, gridsize)
    X, Y = np.meshgrid(x, y)
    wavefront = np.zeros((gridsize, gridsize))
    for l in range(lowest_mode, n_modes+lowest_mode):
        n,m = noll2nm(l)
        wavefront += coefficients[l-lowest_mode]*zernike_cartesian(n, m, X, Y)
    return wavefront

def zernike_polynomials(coefficients, n_modes=10, gridsize=500):
    lowest_mode = 2
    x = np.linspace(-1, 1, gridsize)
    y = np.linspace(-1, 1, gridsize)
    X, Y = np.meshgrid(x, y)
    wavefront = np.zeros((gridsize, gridsize))
    for l in range(lowest_mode, n_modes):
        n,m = noll2nm(l)
        wavefront += coefficients[l-lowest_mode]*zernike_cartesian(n, m, X, Y)
    return wavefront


def create_C_matrix(dm, camera, reference_grid, voltage_step=0.5):
    num_actuators = 19
    len_grid=len(reference_grid)
    C = np.zeros((len_grid*2, num_actuators))
    
    for i in range(num_actuators):
        if i < 19:            
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
            reference_grid, negative_grid = NearestNeighbor(reference_grid, negative_grid)
            
            _, positive_grid = normalize_grids(reference_grid, positive_grid)
            _, negative_grid = normalize_grids(reference_grid, negative_grid)
            slope_x = (positive_grid[:,0] - negative_grid[:,0]) / (2*voltage_step)
            slope_y = (positive_grid[:, 1] - negative_grid[:, 1]) / (2 * voltage_step)

            
            C[:, i] = [slope_x[l // 2] if l % 2 == 0 else slope_y[l // 2] for l in range(len_grid*2)]


        print(f"C col {i}, {len(positive_grid)} pos, {len(negative_grid)} neg, {len_grid} ref")
    np.savetxt("C_matrix.csv", C)
    return C

def create_C_matrix_data(dm, camera, reference_grid, N_datapoints):
    num_actuator = 19
    gridsize = len(reference_grid_normalized)
    slopes_data = np.zeros((N_datapoints, gridsize*2))
    voltages = np.zeros((N_datapoints, num_actuator))
    
    i = 0
    while i < N_datapoints:
        act = np.random.rand(num_actuator) * 2 - 1
        dm.setActuators(act)
        time.sleep(0.1)
        
        slopes = get_current_slopes(reference_grid, sh_camera)
#        print('\nSlopes:', slopes[:10])
        
#        print('\nSlopes', slopes[:10])
#        print(slopes_data.shape)
        
        if len(slopes) == gridsize * 2:
            slopes_data[i, :] = slopes
            voltages[i, :] = act
            i += 1
        
            print(f'Measurement {i} from {N_datapoints}, found {len(slopes)/2} gridpoints')

    np.savetxt("slopes_data.csv", slopes_data)
    np.savetxt("voltages.csv", voltages)
    
    return slopes_data, voltages

    
def calculate_C_matrix(slopes_data, voltages):
    C_matrix, _, _, _ = np.linalg.lstsq(slopes_data, voltages, rcond=None)
#    C_matrix = C_alt.T
    np.savetxt('C_matrix.csv', C_matrix)
    return C_matrix
    
def plot_wavefront(wavefront, reference_grid_normalized, distorted_grid_normalized):
    plt.figure()
    plt.imshow(wavefront, cmap='viridis', origin='lower')
    plt.colorbar()
    plt.scatter((reference_grid_normalized[:, 0]+1)*grid_size//2, (reference_grid_normalized[:, 1]+1)*grid_size//2, s=2, c='red')
    plt.scatter((distorted_grid_normalized[:, 0]+1)*grid_size//2, (distorted_grid_normalized[:, 1]+1)*grid_size//2, s=2, c='black')
    plt.show()
    
def calculate_desired_slope_pattern(zernike_coeffs, reference_grid, n_modes):
    reference_grid, _, _ = normalize_reference_grid_to_unit_circle(reference_grid)
    lowest_mode = 2
    desired_slope_pattern = np.zeros(len(reference_grid) * 2)
    for l in range(lowest_mode, n_modes+lowest_mode):
        n, m = noll2nm(l)
        for k in range(len(reference_grid)):
            x = reference_grid[k, 0]
            y = reference_grid[k, 1]
            desired_slope_pattern[2*k] += zernike_coeffs[l-lowest_mode] * (zernike_cartesian(n, m, x+1e-5, y) - zernike_cartesian(n, m, x, y)) / 1e-5
            desired_slope_pattern[2*k+1] += zernike_coeffs[l-lowest_mode] * (zernike_cartesian(n, m, x, y+1e-5) - zernike_cartesian(n, m, x, y)) / 1e-5
    return desired_slope_pattern

def calculate_desired_voltages(C, slope_pattern):
#    return -slope_pattern @ C
    return C @ -slope_pattern

def get_current_slopes(reference_grid, camera, iteration):
    current_image = camera.grab_frames(4)[-1]
    current_grid = detect_blobs(current_image, index=iteration, show=True)
    current_grid = filter_points_by_neighbors(current_grid)
    
#    current_grid = interpolate_missing_spots(current_grid, reference_grid)
    reference_grid, current_grid = NearestNeighbor(reference_grid, current_grid)
    
    reference_grid_normalized, current_grid_normalized = normalize_grids(reference_grid, current_grid)
    return get_slopes(reference_grid_normalized, current_grid_normalized)

def update_voltages(dm, alpha, current_voltages, voltage_correction):
    new_voltages = current_voltages - alpha * voltage_correction
    new_voltages = np.clip(new_voltages, -1.0, 1.0)
#    print("\nCurrent voltages: ", new_voltages)
    dm.setActuators(new_voltages)
    time.sleep(0.1)
    return new_voltages

def NormalizeVector(vect):
    norm = np.linalg.norm(vect)
    return vect / norm

def converge_to_zernike(dm, camera, reference_grid, C, zernike_coeffs, n_modes, max_iterations=50, tolerance=1e-4, ref_volt=None):
    desired_slope_pattern = calculate_desired_slope_pattern(zernike_coeffs, reference_grid, n_modes)
    print(desired_slope_pattern[:20])
#    print(desired_slope_pattern) # Checked for tip, tilt modes and it seems correct
    
    # desired_voltages = calculate_desired_voltages(C, desired_slope_pattern)
    current_voltages = np.zeros(19)
#    current_voltages = optimized_act
    
    # Commented for now to test
#    if ref_volt is not None:
#        current_voltages[-2:] = ref_volt[-2:]
    dm.setActuators(current_voltages)
    time.sleep(0.1)
#    previous_slopes = np.linalg.pinv(C) @ optimized_act

    for iteration in range(max_iterations):
        current_slopes = get_current_slopes(reference_grid, camera, iteration)
#        current_grid = get_current_grid(camera)
#        reference_grid, current_grid = NearestNeighbor(reference_grid, current_grid)
#        print("N_dots", reference_grid.shape, current_grid.shape)
#        try:
        print(f"1 slope difference: {np.sum(np.abs(desired_slope_pattern - current_slopes))}")
#            unew = ucur - 0.2 * 0.5 * C@s_cur
        voltage_correction = np.clip(calculate_desired_voltages(C, desired_slope_pattern - current_slopes), -1.0, 1.0)
        previous_slopes = current_slopes
            
#        except:
#            print(f"2 slope difference: {np.sum(np.abs(desired_slope_pattern - current_slopes))}")
#            voltage_correction = np.clip(calculate_desired_voltages(C, desired_slope_pattern - previous_slopes), -1.0, 1.0)
            
#        print(f"Voltage correction: {voltage_correction}")
        current_voltages = update_voltages(dm, 0.5, current_voltages, voltage_correction)
        if np.linalg.norm(voltage_correction) < tolerance:
            print(f"Converged after {iteration+1} iterations.")
            break
        
    else:
#        reference_grid_normalized, current_grid_normalized = normalize_grids(reference_grid, current_grid)
        print("Max iterations reached without convergence.")
        
    return current_voltages, current_slopes
    
    


if __name__ == "__main__": 
    plt.close('all')
    with OkoDM(dmtype=1) as dm:
        sh_camera = Camera(camera_index=2, exposure=1)
        normal_camera = Camera(camera_index=1, exposure=0.5)
        optimized_act = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_x.dat")[0]
        n_modes = 15
        
        if False:
            reference_grid = np.loadtxt("reference_grid.csv")
            print("Reference grid loaded from file")
        else:
            reference_grid = create_reference_grid(dm=dm, camera=sh_camera)

        reference_grid_normalized, _, _ = normalize_reference_grid_to_unit_circle(reference_grid)  
        
        if False:
            current_grid = np.loadtxt('interpolated_grid.dat')
            plt.clf()
            plot_displacement(reference_grid, current_grid)
        
        # Decide whether to load from file or generate B and C matrices
        if False:
            B_matrix = np.loadtxt("B_matrix.csv")
            print("B matrix loaded from file")
        else:
            n_modes_B = n_modes
            B_matrix, _ = create_B_matrix(reference_grid_normalized, n_modes_B)
            print(f"B matrix (n={n_modes_B}) has been calculated and saved")
        
        if True:
            slopes_data = np.loadtxt("slopes_data.csv")
            voltages = np.loadtxt("voltages.csv")
            C_matrix = np.loadtxt("C_matrix.csv")            
            print("C matrix loaded from file")
        else:
            slopes_data, voltages = create_C_matrix_data(dm, sh_camera, reference_grid, N_datapoints = 500)
            C_matrix = calculate_C_matrix(slopes_data, voltages)
            print("C_matrix calculated")
            

        '''Testing wavefront reconstruction for given actuator settings'''
        if False:
            rand_act = optimized_act
            rand_act[0] = -0.8
            dm.setActuators(rand_act)
            time.sleep(0.1)
            distorted_grid = get_current_grid(sh_camera)
    
#             Matching grid and normalizing
            reference_grid, distorted_grid = NearestNeighbor(reference_grid, distorted_grid)
            reference_grid_normalized, distorted_grid_normalized = normalize_grids(reference_grid, distorted_grid)
            slopes = get_slopes(reference_grid_normalized, distorted_grid_normalized)
            grid_size = 500
            
            # Plot wavefront of rand_act
            wavefront = reconstruct_wavefront(B_matrix, slopes, n_modes=n_modes, gridsize=grid_size)
            plot_wavefront(wavefront, reference_grid_normalized, distorted_grid_normalized)
            plt.show()
            
        '''Testing C matrix construction'''
        if False:
            for ii, slopes in enumerate(slopes_data[:10]):
                print()
                print(slopes @ C_matrix.T)
                print(voltages[ii])
            
            
            
        """Testing convergence algorithm"""
        if False:
            grid_size = 500

            zernike_coeffs = np.zeros(n_modes)
            index = 1
            zernike_coeffs[index] = 0.8e-1 if index <2 else 3e-2
            
            desired_slopes = calculate_desired_slope_pattern(zernike_coeffs, reference_grid_normalized, n_modes)
            desired_slopes = np.reshape(desired_slopes, (-1, 2))
            
            wavefront = reconstruct_wavefront(B_matrix, desired_slopes.flatten(), n_modes=n_modes, gridsize=grid_size)
            plot_wavefront(wavefront, reference_grid_normalized, reference_grid_normalized+desired_slopes)
            plt.show()
             
            
            
        
        '''Control mirror to achieve wf corresponding to desired zernike coeffs'''
        if True:
            zernike_coeffs = np.zeros(n_modes)
            index = 0
            zernike_coeffs[index] = 0.8e-1 if index <2 else 3e-2 # Normalize coefficients? #TODO!
            
            voltages, slopes = converge_to_zernike(dm, sh_camera,   reference_grid,\
                                                                    C_matrix, \
                                                                    zernike_coeffs,\
                                                                    n_modes, \
                                                                    ref_volt = optimized_act)
            
            dm.setActuators(voltages)
            time.sleep(0.1)
            
            slopes = get_current_slopes(reference_grid, sh_camera)
            print("\n\n", slopes)
            
            grid_size = 500
            wavefront = reconstruct_wavefront(B_matrix[:, :len(slopes)], slopes, n_modes=n_modes, gridsize=grid_size)
            plot_wavefront(wavefront, reference_grid_normalized, reference_grid_normalized+np.reshape(slopes, (-1, 2)))
            
