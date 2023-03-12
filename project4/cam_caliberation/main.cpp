//
//  main.cpp
//  project4
//
//  Created by Yizhou Li on 3/9/23.
//

#include <iostream>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "utils.hpp"

using namespace std;
using namespace cv;

int main(int argc, const char * argv[]) {
    std::filesystem::create_directory("./caliberation_images/");
    
    VideoCapture cap(0);
    if(!cap.isOpened()) {
        printf("Unable to open video device");
        return 0;
    }
    
    namedWindow("video");
    Mat src, dst;
    char lastKey = 'n';
    
    // initialize some variables
    Mat tmp_caliberation_img;
    vector<Point2f> corners;
    vector<vector<Point2f>> corner_list;
    vector<Vec3f> points;
    vector<vector<Vec3f>> point_list;
    Size patternsize(6, 9); //interior number of corners
    for (int i = 0; i > -patternsize.width; i--) {
        for (int j = 0; j < patternsize.height; j++) {
            points.push_back(Vec3f(j, i, 0));
        }
    }
    
    while (true) {
        cap >> src;
        dst = src.clone();
        if(src.empty()) {
            printf("src is empty\n");
            break;
        }
        
        // see if there is a waiting keystroke
        char key = waitKey(25);
        if (key == -1) {
            key = lastKey;
        }
        
        if (key == 'q') {
            break;
        } else if (key == '1') {
            Mat gray;
            cvtColor(src, gray, COLOR_BGR2GRAY);
            //CALIB_CB_FAST_CHECK saves a lot of time on images that do not contain any chessboard corners
            vector<Point2f> tmp_corners;
            bool patternfound = findChessboardCorners(gray, patternsize, tmp_corners,
                                                      CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                      + CALIB_CB_FAST_CHECK);
            if (patternfound) {
                cornerSubPix(gray, tmp_corners, Size(11, 11), Size(-1, -1),
                             TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1));
                drawChessboardCorners(dst, patternsize, Mat(tmp_corners), patternfound);
                printf("the total number of corners: %lu\n", tmp_corners.size());
                printf("the first corner: (%f, %f)\n", tmp_corners[0].x, tmp_corners[0].y);
                corners = tmp_corners;
                tmp_caliberation_img = src.clone();
            }
            
            lastKey = 'n';
        } else if (key == '2') {
            if (corners.size() != 0) {
                corner_list.push_back(corners);
                point_list.push_back(points);
                imwrite("./caliberation_images/" + to_string(corner_list.size()) + ".jpg", tmp_caliberation_img);
            }

            lastKey = 'n';
        } else if (key == '3') {
            if (corner_list.size() >= 5) {
                Mat cameraMatrix = Mat::zeros(3, 3, CV_64FC1);
                cameraMatrix.at<double>(0, 0) = 0;
                cameraMatrix.at<double>(0, 1) = 0;
                cameraMatrix.at<double>(0, 2) = 1;
                cameraMatrix.at<double>(1, 0) = 0;
                cameraMatrix.at<double>(1, 1) = 1;
                cameraMatrix.at<double>(1, 2) = src.rows/2;
                cameraMatrix.at<double>(2, 0) = 1;
                cameraMatrix.at<double>(2, 1) = 0;
                cameraMatrix.at<double>(2, 2) = src.cols/2;
                
                Mat distCoeffs, rvecs, tvecs, std_deviations_intrinsic, std_deviations_extrinsic, per_view_errors;
                // print some variables before the camera caliberation
                for(int i = 0; i < cameraMatrix.rows; i++) {
                    for(int j = 0; j < cameraMatrix.cols; j++) {
                        printf("camera matrix %f\n", cameraMatrix.at<double>(i, j));
                    }
                }
                for(int i = 0; i < distCoeffs.rows; i++) {
                    for(int j = 0; j < distCoeffs.cols; j++) {
                        printf("distortion coeeficients %f\n", distCoeffs.at<double>(i, j));
                    }
                }
                
                // do camera caliberation
                double reprojection_error = calibrateCamera(point_list, corner_list, src.size(), cameraMatrix, distCoeffs, rvecs, tvecs, std_deviations_intrinsic, std_deviations_extrinsic, per_view_errors);
                
                // print some variables after the camera caliberation
                for(int i = 0; i < cameraMatrix.rows; i++) {
                    for(int j = 0; j < cameraMatrix.cols; j++) {
                        printf("camera matrix %f\n", cameraMatrix.at<double>(i, j));
                    }
                }
                for(int i = 0; i < distCoeffs.rows; i++) {
                    for(int j = 0; j < distCoeffs.cols; j++) {
                        printf("distortion coeeficients %f\n", distCoeffs.at<double>(i, j));
                    }
                }
                printf("the reprojection error is: %f\n", reprojection_error);
                save_cam_caliberation_info(cameraMatrix, distCoeffs, rvecs, tvecs);
            } else {
                printf("You have to specifiy atleast 5 images for camera caliberation.");
            }
            printf("currently has %lu pre-trained images\n", corner_list.size());
            
            lastKey = 'n';
        }
        
        imshow("video", dst);
    }
    
    // todo: potentially destroy data created in program running
//    std::filesystem::remove_all("./caliberation_images/");
    return 0;
}
