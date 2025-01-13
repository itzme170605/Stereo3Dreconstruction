# 3D Reconstruction from Stereo Cameras

This project implements a 3D reconstruction system using stereo vision techniques. It computes a 3D point cloud from stereo images using depth estimation and rectification methods.

---

## Features
- **Stereo Image Rectification**: Aligns the left and right images to correct distortions and align image planes.
- **Depth Map Calculation**: Uses Semi-Global Block Matching (SGBM) to calculate the disparity map.
- **3D Point Cloud Generation**: Reprojects the disparity map into 3D space.
- **Point Cloud Export**: Saves the 3D point cloud in PLY format for visualization in tools like MeshLab.

---

## Requirements

### Libraries
- OpenCV 4.x or higher

Install OpenCV using:
```bash
sudo apt-get install libopencv-dev
```

### Build Tools
- A C++17-compatible compiler (e.g., GCC, Clang, MSVC).

---

## Project Structure
- `path/to/left_image.jpg`: Left image for stereo vision.
- `path/to/right_image.jpg`: Right image for stereo vision.
- `point_cloud.ply`: Output 3D point cloud file.

---

## How to Run

### 1. Replace Calibration Parameters
Replace the intrinsic (`K`) and distortion (`D`) parameters, along with the rotation (`R`) and translation (`T`), with values obtained from your stereo camera calibration.

### 2. Compile the Code
Save the code as `stereo_3d_reconstruction.cpp` and compile it using a compatible C++ compiler:

```bash
g++ stereo_3d_reconstruction.cpp -o stereo_3d_reconstruction `pkg-config --cflags --libs opencv4`
```

### 3. Run the Program
Provide the paths to the left and right stereo images:

```bash
./stereo_3d_reconstruction
```

### 4. View the Point Cloud
Open the `point_cloud.ply` file in a visualization tool like MeshLab or CloudCompare to inspect the reconstructed 3D scene.

---

## Example Calibration Parameters
Replace these example values in the code:
- Intrinsic Matrix (`K`):
  ```
  Mat K = (Mat_<double>(3, 3) << 700, 0, 320, 0, 700, 240, 0, 0, 1);
  ```
- Distortion Coefficients (`D`):
  ```
  Mat D = (Mat_<double>(1, 5) << 0.1, -0.25, 0.001, 0.001, 0.1);
  ```
- Rotation Matrix (`R`) and Translation Vector (`T`):
  ```
  Mat R = Mat::eye(3, 3, CV_64F);
  Mat T = (Mat_<double>(3, 1) << 0.1, 0, 0);
  ```

---

## Output
- **Point Cloud**: The output `point_cloud.ply` contains the 3D points reconstructed from the stereo images.

---

## Notes
1. Ensure the stereo images are properly captured and aligned.
2. Tune the SGBM parameters for optimal depth map quality.
3. Use high-resolution images for better reconstruction accuracy.

---

## Future Enhancements
- Implement additional stereo algorithms for disparity calculation.
- Use GPU acceleration for faster processing.
- Automate calibration and parameter tuning.

---

