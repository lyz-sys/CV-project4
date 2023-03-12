//
//  main.cpp
//  cam_extra
//
//  Created by Yizhou Li on 3/11/23.
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
    VideoCapture cap(1);
    if(!cap.isOpened()) {
        printf("Unable to open video device");
        return 0;
    }
    
    namedWindow("video");
    Mat src, dst;
    char lastKey = 'n';
    
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
        } else if (key == '4') {
            Mat camera_matrix, dist_coefficients, rvecs, tvecs;
            read_cam_caliberation_info(camera_matrix, dist_coefficients, rvecs, tvecs);
            
            Mat gray;
            cvtColor(src, gray, COLOR_BGR2GRAY);
            
            Size patternsize(6, 9); //interior number of corners
            vector<Point2f> corners;
            vector<Vec3f> points;
            for (int i = 0; i > -patternsize.width; i--) {
                for (int j = 0; j < patternsize.height; j++) {
                    points.push_back(Vec3f(j, i, 0));
                }
            }
            
            bool patternfound = findChessboardCorners(gray, patternsize, corners,
                                                      CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                      + CALIB_CB_FAST_CHECK);
            if (patternfound) {
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                             TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1));
                drawChessboardCorners(dst, patternsize, Mat(corners), patternfound);
                printf("the total number of corners: %lu\n", corners.size());
                printf("the first corner: (%f, %f)\n", corners[0].x, corners[0].y);
                
                solvePnP(points, corners, camera_matrix, dist_coefficients, rvecs, tvecs);
                
                // print rvecs and tvecs in realtime
                for(int i = 0; i < rvecs.rows; i++) {
                    for(int j = 0; j < rvecs.cols; j++) {
                        printf("rvecs: %f\n", rvecs.at<double>(i, j));
                    }
                }
                for(int i = 0; i < tvecs.rows; i++) {
                    for(int j = 0; j < tvecs.cols; j++) {
                        printf("tvecs: %f\n", tvecs.at<double>(i, j));
                    }
                }
            }
            
            lastKey = 'n';
        } else if (key == '5') {
            Mat camera_matrix, dist_coefficients, rvecs, tvecs;
            read_cam_caliberation_info(camera_matrix, dist_coefficients, rvecs, tvecs);
            
            Mat gray;
            cvtColor(src, gray, COLOR_BGR2GRAY);
            
            Size patternsize(6, 9); //interior number of corners
            vector<Point2f> corners;
            vector<Vec3f> points;
            for (int i = 0; i > -patternsize.width; i--) {
                for (int j = 0; j < patternsize.height; j++) {
                    points.push_back(Vec3f(j, i, 0));
                }
            }
            
            bool patternfound = findChessboardCorners(gray, patternsize, corners,
                                                      CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                      + CALIB_CB_FAST_CHECK);
            if (patternfound) {
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                             TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1));
                drawChessboardCorners(dst, patternsize, Mat(corners), patternfound);
                printf("the total number of corners: %lu\n", corners.size());
                printf("the first corner: (%f, %f)\n", corners[0].x, corners[0].y);
                
                solvePnP(points, corners, camera_matrix, dist_coefficients, rvecs, tvecs);
                
                // get four corners of the chessboard, and draw circles accordingly
                // todo: after camer caliberated; add four more corner points
                vector<Point2f> img_points;
                vector<Vec3f> obj_points;
                obj_points.push_back(Vec3f(5, -5, 0));
                
                projectPoints(obj_points, rvecs, tvecs, camera_matrix, dist_coefficients, img_points);
                circle(dst, img_points[0], 50, Scalar(255, 0, 0));
            }
            
            lastKey = '5';
        } else if (key == '6') {
            
            lastKey = '6';
        }
        
        imshow("video", dst);
    }
    return 0;
}
