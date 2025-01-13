#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <string>
#include <fstream>

using namespace cv;
using namespace std;

int main() {
    // Paths to stereo image pairs
    string left_image_path = "path/to/left_image.jpg";
    string right_image_path = "path/to/right_image.jpg";

    // Camera calibration parameters (example values, replace with actual calibration results)
    Mat K = (Mat_<double>(3, 3) << 700, 0, 320, 0, 700, 240, 0, 0, 1); // Intrinsic matrix
    Mat D = (Mat_<double>(1, 5) << 0.1, -0.25, 0.001, 0.001, 0.1);      // Distortion coefficients
    Mat R = Mat::eye(3, 3, CV_64F);                                     // Rotation matrix
    Mat T = (Mat_<double>(3, 1) << 0.1, 0, 0);                          // Translation vector

    // Step 1: Load stereo images
    Mat left_image = imread(left_image_path, IMREAD_GRAYSCALE);
    Mat right_image = imread(right_image_path, IMREAD_GRAYSCALE);

    if (left_image.empty() || right_image.empty()) {
        cerr << "Error: Unable to load images. Check the paths." << endl;
        return -1;
    }

    // Step 2: Rectify images
    Size image_size = left_image.size();
    Mat R1, R2, P1, P2, Q;
    stereoRectify(K, D, K, D, image_size, R, T, R1, R2, P1, P2, Q);

    Mat map1_left, map2_left, map1_right, map2_right;
    initUndistortRectifyMap(K, D, R1, P1, image_size, CV_16SC2, map1_left, map2_left);
    initUndistortRectifyMap(K, D, R2, P2, image_size, CV_16SC2, map1_right, map2_right);

    Mat rectified_left, rectified_right;
    remap(left_image, rectified_left, map1_left, map2_left, INTER_LINEAR);
    remap(right_image, rectified_right, map1_right, map2_right, INTER_LINEAR);

    // Step 3: Compute the depth map
    Ptr<StereoSGBM> stereo = StereoSGBM::create(0, 16 * 5, 9);
    stereo->setP1(8 * 3 * 9 * 9);
    stereo->setP2(32 * 3 * 9 * 9);
    stereo->setMinDisparity(0);
    stereo->setNumDisparities(16 * 5);
    stereo->setBlockSize(9);
    stereo->setDisp12MaxDiff(1);
    stereo->setUniquenessRatio(15);
    stereo->setSpeckleWindowSize(100);
    stereo->setSpeckleRange(32);

    Mat disparity;
    stereo->compute(rectified_left, rectified_right, disparity);
    disparity.convertTo(disparity, CV_32F, 1.0 / 16.0);

    // Step 4: Generate a 3D point cloud
    Mat points_3D;
    reprojectImageTo3D(disparity, points_3D, Q, true);

    // Save the point cloud
    ofstream point_cloud_file("point_cloud.ply");
    point_cloud_file << "ply\n";
    point_cloud_file << "format ascii 1.0\n";
    point_cloud_file << "element vertex " << countNonZero(disparity > disparity.min()) << "\n";
    point_cloud_file << "property float x\n";
    point_cloud_file << "property float y\n";
    point_cloud_file << "property float z\n";
    point_cloud_file << "end_header\n";

    for (int y = 0; y < points_3D.rows; y++) {
        for (int x = 0; x < points_3D.cols; x++) {
            Vec3f point = points_3D.at<Vec3f>(y, x);
            if (disparity.at<float>(y, x) > disparity.min()) {
                point_cloud_file << point[0] << " " << point[1] << " " << point[2] << "\n";
            }
        }
    }

    point_cloud_file.close();
    cout << "3D point cloud saved as 'point_cloud.ply'" << endl;

    return 0;
}
