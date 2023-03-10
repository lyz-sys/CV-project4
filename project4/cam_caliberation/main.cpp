//
//  main.cpp
//  project4
//
//  Created by Yizhou Li on 3/9/23.
//

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, const char * argv[]) {
    VideoCapture cap(1); // 0 for IPhone camer, and 1 for built in Mac webcam
    if(!cap.isOpened()) {
        printf("Unable to open video device");
        return 0;
    }
    
    namedWindow("video");
    Mat src, dst;
    char lastKey = 's';
    
    // initialize some variables
    vector<Point2f> corners;
    vector<vector<Point2f>> corner_list;
    vector<Vec3f> points;
    vector<vector<Vec3f>> point_list;
    Size patternsize(6, 9); //interior number of corners
    for (int i = 0; i > -patternsize.width; i--) {
        for (int j = 0; j < patternsize.height; j++) {
            points.push_back(Vec3f(i, j, 0));
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
                printf("the first corner: (%f, %f)", tmp_corners[0].x, tmp_corners[0].y);
                corners = tmp_corners;
            }
            lastKey = '1';
        } else if (key == '2') {
            if (corners.size() != 0) {
                corner_list.push_back(corners);
                point_list.push_back(points);
            }
            
            lastKey = 'n';
        }
        
        imshow("video", dst);
    }
    
    return 0;
}
