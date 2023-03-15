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
    // get some properties of the image
    cv::Size refS((int) cap.get(cv::CAP_PROP_FRAME_WIDTH),
                  (int) cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);
    namedWindow("video");
    
    Size patternsize(9, 6); //interior number of corners
    Mat src, dst, gray, camera_matrix, dist_coefficients, rvecs, tvecs;
    read_cam_caliberation_info(camera_matrix, dist_coefficients);

    while (true) {
        cap >> src;
        dst = src.clone();
        if (src.empty()) {
            printf("src is empty\n");
            break;
        }
  
        cvtColor(src, gray, COLOR_BGR2GRAY);
 
        vector<Point2f> corners;
        vector<Point3f> points;
        for (int i = 0; i < patternsize.height; i++) {
            for (int j = 0; j < patternsize.width; j++) {
                points.push_back(Point3f(j, -i, 0));
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
            cout << "rvecs:" << endl;
            cout << rvecs << endl;
            cout << "tvecs:" << endl;
            cout << tvecs << endl;
        }
        
        // see if there is a waiting keystroke
        char key = waitKey(25);
        
        if (key == 'q') {
            break;
        } else if (key == '5') {
            if (patternfound) {
                vector<Point2f> img_points;
                vector<Point3f> obj_points;
                obj_points.push_back(Point3f(0, 0, 0));
                obj_points.push_back(Point3f(10, 0, 0));
                obj_points.push_back(Point3f(0, -10, 0));
                obj_points.push_back(Point3f(0, 0, 10));
                projectPoints(obj_points, rvecs, tvecs, camera_matrix, dist_coefficients, img_points);
                line(dst, Point(img_points[0].x, img_points[0].y), Point(img_points[1].x, img_points[1].y), Scalar(255, 0, 0), 5);
                line(dst, Point(img_points[0].x, img_points[0].y), Point(img_points[2].x, img_points[2].y), Scalar(255, 0, 0), 5);
                line(dst, Point(img_points[0].x, img_points[0].y), Point(img_points[3].x, img_points[3].y), Scalar(255, 0, 0), 5);
            }
        } else if (key == '6') {
            
        }
        
        imshow("video", dst);
    }
    return 0;
}
