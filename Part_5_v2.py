import cv2
import numpy as np
from scipy.spatial import distance_matrix
from scipy.spatial import cKDTree
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
from skimage.filters import threshold_triangle, threshold_otsu, threshold_local
from skimage.transform import radon, rotate

from skimage.feature import canny
from skimage.transform import hough_line, hough_line_peaks

from sklearn.linear_model import LinearRegression
from pyueye import ueye
from camera.ueye_camera import uEyeCamera
from dm.okotech.dm import OkoDM
import time
import os
from scipy.fftpack import fft2, fftshift
from zernike import RZern

# Set the path for DLL loading
os.environ['PATH'] = "C:\\AO-course-2024\\dm\okotech\\okodm_sdk\\python" + os.pathsep + os.environ['PATH']



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


#class Camera:
#    def __init__(self, camera_index, exposure):
#        self.camera_index = camera_index
#        self.exposure = exposure
#        self.cam = uEyeCamera(device_id=self.camera_index)
#        self.initialize_camera()
#
#    def initialize_camera(self):
#        self.cam.set_colormode(ueye.IS_CM_MONO8)
#        self.cam.alloc(buffer_count=10)
#        self.cam.set_exposure(self.exposure)
#        self.cam.capture_video(True)
#
#    def release_camera(self):
#        if self.cam is not None:
#            self.cam.stop_video()
#            self.cam.exit()
#            self.cam = None
#
#    def grab_frames(self, nframes):
#        imgs = np.zeros((nframes, 1024, 1280), dtype=np.uint8)
#        acquired = 0
#        while acquired < nframes:
#            frame = self.cam.grab_frame()
#            if frame is not None:
#                imgs[acquired] = frame
#                acquired += 1
#        return imgs





class DeformableMirror:
    def __init__(self, dm_type):
        self.dm_type = dm_type

    def setActuators(self, act):
        with OkoDM(dmtype=self.dm_type) as dm:
            dm.setActuators(act)
            time.sleep(0.1)


def threshold_image(img):
    thresh = threshold_otsu(img)
    _, threshimg = cv2.threshold(img, thresh, 255, cv2.THRESH_TOZERO)
    return threshimg

def find_centroid(img):
    img = np.array(img)
    M = cv2.moments(img)
    x0 = int(M["m10"] / M["m00"])
    y0 = int(M["m01"] / M["m00"])
    return x0, y0

def adaptive_threshold_image(img):
    adaptive_thresh = threshold_local(img, block_size=51, offset=10)
    binary_img = img > adaptive_thresh
    return binary_img.astype(np.uint8) * 255


def detect_blobs(img, show=False):
    thresh = threshold_triangle(img)
    thresh = threshold_otsu(img)
    _, binaryimg = cv2.threshold(img, thresh, 255, cv2.THRESH_TOZERO)
#    binaryimg = adaptive_threshold_image(img)
    params = cv2.SimpleBlobDetector_Params()
    # Filter by Area.
    params.filterByArea = True
    params.blobColor = 255
    params.minArea = 25
    params.maxArea = 1000
    
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
    
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5
    
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(binaryimg)
    keypoints = merge_close_keypoints(keypoints)
    if show:
        im_with_keypoints = cv2.drawKeypoints(binaryimg, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        plt.imshow(im_with_keypoints)
    return np.array([kp.pt for kp in keypoints])


def find_rotation_angle(img):
#    xcoords = np.array([pos[0] for pos in dot_positions]).reshape(-1, 1)
#    ycoords = np.array([pos[1] for pos in dot_positions])
#    model = LinearRegression().fit(xcoords, ycoords)
#    angle = np.arctan(model.coef_[0])
    img = np.array(img)
    theta = np.linspace(0., 180., np.max(img.shape), endpoint=False)
    sinogram = radon(img, theta=theta)
    
    rotation_angle = np.argmax(np.sum(sinogram, axis=0))
    if rotation_angle > 90:
        rotation_angle -= 180  # Adjust angle to be within -90 to 90 degrees

    return rotation_angle


def calculate_rotation_angle_v2(image):
    edges = canny(np.array(image), sigma=3)
    h, theta, d = hough_line(edges) 
    accum, angles, dists = hough_line_peaks(h, theta, d)
    mode_angle = np.rad2deg(np.median(angles))
    rotation_adjustment = (mode_angle + 90) % 180
    return rotation_adjustment


def rotate_img(img, angle):
    h, w = img.shape
    center = find_centroid(img)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    newimg = cv2.warpAffine(img, M, (w, h))
    return newimg

def crop_to_center(img, keypoints, center):
    if not keypoints:
        return img  # Return the original image if no keypoints are detected
    
    # Calculate the radius based on the farthest keypoint from the center
    radius = max(np.linalg.norm(np.array(kp) - np.array(center)) for kp in keypoints)

    # Define the side length of the square to crop
    side_length = int(2 * radius)

    # Calculate the top-left corner of the square crop
    top_left_x = max(int(center[0] - radius), 0)
    top_left_y = max(int(center[1] - radius), 0)

    # Adjust the bottom right corner to keep within image bounds and ensure the crop is square
    if top_left_x + side_length > img.shape[1]:
        top_left_x = img.shape[1] - side_length
    if top_left_y + side_length > img.shape[0]:
        top_left_y = img.shape[0] - side_length

    # Crop the image to the defined square
    cropped_img = img[top_left_y:top_left_y + side_length, top_left_x:top_left_x + side_length]
    return cropped_img


def draw_grid(img, keypoints):
    keypoints = np.array(keypoints, dtype=np.float32)
    img_with_rectangles = img.copy()

    # Calculate the average distance between keypoints
    if len(keypoints) > 1:
        distances = np.sqrt((keypoints[:, None, 0] - keypoints[None, :, 0])**2 + 
                            (keypoints[:, None, 1] - keypoints[None, :, 1])**2)
        avg_distance = np.mean(distances[distances > 0])
    else:
        avg_distance = 20  # Default value if there is only one keypoint

    # Draw rectangles around each keypoint
    half_side = int(avg_distance / 2)
    for (x, y) in keypoints:
        top_left = (int(x - half_side), int(y - half_side))
        bottom_right = (int(x + half_side), int(y + half_side))
        cv2.rectangle(img_with_rectangles, top_left, bottom_right, (150, 150, 255), 1)

    return img_with_rectangles

def draw_grid(img, keypoints):
    keypoints = np.array(keypoints, dtype=np.float32)
    img_with_rectangles = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)  # Convert to BGR for colored rectangles

    if len(keypoints) > 1:
        distances = np.sqrt((keypoints[:, None, 0] - keypoints[None, :, 0])**2 + 
                            (keypoints[:, None, 1] - keypoints[None, :, 1])**2)
        avg_distance = np.mean(distances[distances > 0])
    else:
        avg_distance = 20

    half_side = int(avg_distance / 2)
    color = (0, 255, 0)  # Green color for the grid
    for (x, y) in keypoints:
        top_left = (int(x - half_side), int(y - half_side))
        bottom_right = (int(x + half_side), int(y + half_side))
        if 0 <= top_left[0] < img.shape[1] and 0 <= top_left[1] < img.shape[0] and \
           0 <= bottom_right[0] < img.shape[1] and 0 <= bottom_right[1] < img.shape[0]:
            cv2.rectangle(img_with_rectangles, top_left, bottom_right, color, 1)

    return img_with_rectangles

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
    optimized_act = np.loadtxt(f"C:\\AO-course-2024\\part_4\\last_x.dat")[0]
    dm.setActuators(optimized_act)
    time.sleep(0.2)
    img = sh_camera.grab_frames(4)[-1]
    
    return img
    

def plot(img):
    plt.clf()
    plt.imshow(img, aspect='auto', interpolation='bicubic')
#    plt.colorbar()
    

def plot_displacement(reference_dots, random_dots, indices):
    plt.scatter(reference_dots[:, 0],reference_dots[:, 1], 5,  c='red')
    plt.scatter(random_dots[:,0],random_dots[:,1], 5, c='green')
    
    if len(reference_dots) > len(random_dots):
       for i, dot in enumerate(random_dots):
           plt.plot([reference_dots[indices[i], 0], random_dots[i, 0]], [reference_dots[indices[i], 1], random_dots[i, 1]], '-', c='black')
            
    else:
        for i, dot in enumerate(reference_dots):
            plt.plot([reference_dots[indices[i], 0], random_dots[i, 0]], [reference_dots[indices[i], 1], random_dots[i, 1]], '-', c='black')
        
    

def compute_fft_image(image):
    fft_result = fft2(image)
    fft_magnitude = np.abs(fftshift(fft_result))
    return fft_magnitude

#def NearestNeighbor(reference_dots, random_dots):
#    all_dots = np.concatenate((reference_dots, random_dots))
#    
#    dist = np.linalg.norm(reference_dots-random_dots)
#
#    
#    neigh = NearestNeighbors(1, .4)
#    neigh.fit(random_dots)
#        
#    _, indices = neigh.kneighbors(reference_dots, 1)
#    
#    dist_x = [dot[0] - dot[indices[0]] for dot in all_dots]
#    dist_y = [dot[1] - dot[indices[1]] for dot in all_dots]
#    
#    return np.hstack((dist_x, dist_y))

def NearestNeighbor(reference_dots, random_dots):
#    all_dots = np.concatenate((reference_dots, random_dots))
#    
#    all_dist = [np.sqrt((reference_dots[:, 0]-random_dots[:, 0])**2 + (reference_dots[:, 1]-random_dots[:, 1])**2)]
#    indices = np.where(all_dist == np.min(all_dist, axis=0))
#    
#
#    return indices
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
    displacement = reference_dots - random_dots[indices]
    return displacement



if __name__ == "__main__":
    with OkoDM(dmtype=1) as dm:
        sh_camera = Camera(camera_index=2, exposure=3)
        normal_camera = Camera(camera_index=1, exposure=1)
        try:
    #        dm = DeformableMirror(dm_type=1)
        #    img = sh_camera.grab_frames(4)[-1]
            reference_img_normal = reference_img(dm, normal_camera)
            reference_img = reference_img(dm, sh_camera)
            reference_dots = detect_blobs(reference_img, show=True)
    #        plt.show()
    #        print(reference_dots)
            dm.setActuators(np.random.rand(19)*2-1)
            time.sleep(0.2)
            random_img = sh_camera.grab_frames(4)[-1]
            random_dots = detect_blobs(random_img, show=False)
            reference_dots = filter_points_by_neighbors(reference_dots)
            random_dots = filter_points_by_neighbors(random_dots)
            print(len(reference_dots), len(random_dots))
            plt.show() 
        finally:
            pass
#            sh_camera.release_camera()
#            normal_camera.release_camera()
            
    
#    NN_dist = NearestNeighbor(reference_dots, random_dots)
    
#    plot(reference_img)
    
    
#    rotation_angle = calculate_rotation_angle_v2(img)
#    print(f"Angle: {rotation_angle} deg")
#    img_rotated = rotate_img(img, rotation_angle)
#        
#    dot_positions = detect_blobs(img_rotated,) #show=True)
#
#    center = find_centroid(img_rotated)
#    img_cropped = crop_to_center(img_rotated, dot_positions, center)
#    dot_positions = detect_blobs(img_cropped, show=True)
#    plt.show()
#    print(dot_positions)
#
#    
#    img_with_grid = draw_grid(img_cropped, dot_positions) 
#    plot(img_with_grid)
    # FFT
#    fft_img = compute_fft_image(img_cropped)
#    plot(np.log1p(fft_img))
#    rotation_angle = calculate_rotation_angle_v2(fft_img)
#    print(f"FFT Angle: {rotation_angle} deg")
#    plot(img_cropped)

#    plt.show()



#%%
plt.clf()
indices = NearestNeighbor(reference_dots, random_dots)

plot_displacement(reference_dots, random_dots, indices)
        

#%%
        
zern = RZern(15)
all_dots = np.concatenate((reference_dots, random_dots))


displacements = calculate_displacement(reference_dots, random_dots, indices)

ddx = np.arange(np.min(all_dots[:, 0]), np.max(all_dots[:, 0]))
ddy = np.arange(np.min(all_dots[:, 1]), np.max(all_dots[:, 1]))

ddx = 2*ddx/np.linalg.norm(ddx) -1
ddy = 2*ddy/np.linalg.norm(ddy) -1

xv, yv = np.meshgrid(ddx, ddy)



#max_dist = np.max(np.linalg.norm(reference_dots, axis=1))

zern.make_cart_grid(xv, yv)

c0 = np.random.normal(size=zern.nk)
phi = zern.eval_grid(c0)
#
coeffs = zern.fit_cart_grid(phi)[0]


#Phi = zern.eval_grid()