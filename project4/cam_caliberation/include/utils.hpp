//
//  utils.hpp
//  project4
//
//  Created by Yizhou Li on 3/9/23.
//

#ifndef utils_hpp
#define utils_hpp

#include <stdio.h>
#include <opencv2/core.hpp>

using namespace cv;

void save_cam_caliberation_info(Mat camera_matrix, Mat dist_coefficients, Mat rvecs, Mat tvecs);
void read_cam_caliberation_info(Mat &camera_matrix, Mat &dist_coefficients, Mat &rvecs, Mat &tvecs);

#endif /* utils_hpp */
