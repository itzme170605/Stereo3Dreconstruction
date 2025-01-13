import cv2
import numpy as np
import open3d as o3d

# Paths to stereo image pairs
left_image_path = "path/to/left_image.jpg"
right_image_path = "path/to/right_image.jpg"

# Camera calibration parameters (example values, replace with actual calibration results)
K = np.array([[700, 0, 320], [0, 700, 240], [0, 0, 1]])  # Intrinsic matrix
D = np.array([0.1, -0.25, 0.001, 0.001, 0.1])            # Distortion coefficients
R = np.eye(3)                                            # Rotation matrix
T = np.array([0.1, 0, 0])                                # Translation vector

# Step 1: Load stereo images
left_image = cv2.imread(left_image_path, cv2.IMREAD_GRAYSCALE)
right_image = cv2.imread(right_image_path, cv2.IMREAD_GRAYSCALE)

if left_image is None or right_image is None:
    raise ValueError("Failed to load images. Check the paths.")

# Step 2: Rectify images
h, w = left_image.shape[:2]
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K, D, K, D, (w, h), R, T)

map1_left, map2_left = cv2.initUndistortRectifyMap(K, D, R1, P1, (w, h), cv2.CV_16SC2)
map1_right, map2_right = cv2.initUndistortRectifyMap(K, D, R2, P2, (w, h), cv2.CV_16SC2)

rectified_left = cv2.remap(left_image, map1_left, map2_left, cv2.INTER_LINEAR)
rectified_right = cv2.remap(right_image, map1_right, map2_right, cv2.INTER_LINEAR)

# Step 3: Compute the depth map
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=16 * 5,  # Must be divisible by 16
    blockSize=9,
    P1=8 * 3 * 9**2,
    P2=32 * 3 * 9**2,
    disp12MaxDiff=1,
    uniquenessRatio=15,
    speckleWindowSize=100,
    speckleRange=32
)
disparity = stereo.compute(rectified_left, rectified_right).astype(np.float32) / 16.0

# Step 4: Generate a 3D point cloud
points_3D = cv2.reprojectImageTo3D(disparity, Q)
mask = disparity > disparity.min()
points_3D = points_3D[mask]
colors = cv2.cvtColor(cv2.imread(left_image_path), cv2.COLOR_BGR2RGB)
colors = colors[mask]

# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_3D)
pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

# Step 5: Visualize the point cloud
o3d.visualization.draw_geometries([pcd])

# Save the point cloud
o3d.io.write_point_cloud("point_cloud.ply", pcd)
print("3D point cloud saved as 'point_cloud.ply'")
