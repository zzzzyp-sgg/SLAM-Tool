/*******************************************************
 * Copyright (C) 2023, WuHan University
 * 
 * This file is just for development, and cannot be used yet.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: zyp
 *******************************************************/

#include "../qr_code/qr_math.h"

/// reconstruct form H matrix(from ORB-SLAM)
void FindH(const VecPoint2f &kp1, const VecPoint2f &kp2, float &score, cv::Mat &H21, VecBool &inlierH);

/// compute H matrix
cv::Mat ComputeH(const VecPoint2f &vP1, const VecPoint2f &vP2);

/// normalize
void Normalize(const VecPoint2f &kp, VecPoint2f &kpn, cv::Mat &T);

/// check H score
float CheckHomography(const cv::Mat &H21, const cv::Mat &H12,
                        const VecPoint2f &kp1, const VecPoint2f &kp2, float sigma, VecBool current_in);

/// reconstruct H matrix
bool ReconstructH(VecBool &inlierH, cv::Mat &H21, cv::Mat &K, const VecPoint2f &kp1, const VecPoint2f &kp2,
                    cv::Mat &R21, cv::Mat &t21, VecPoint3d &points, VecBool &vbTriangulated, float minParallax, int minTriangulated);

/// check R and t matrix
int CheckRT(const cv::Mat &R, const cv::Mat &t, const VecPoint2f &vKeys1, const VecPoint2f &vKeys2,
            VecBool &vbMatchesInliers, const cv::Mat &K, VecPoint3d &vP3D, float th2, VecBool &vbGood, float &parallax);

/// triangulate from ORB-SLAM
void Triangulate(const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);