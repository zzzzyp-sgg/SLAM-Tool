/*******************************************************
 * Copyright (C) 2023, WuHan University
 * 
 * This file is for modified VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: zyp
 *******************************************************/

#ifndef QR_MATH_H
#define QR_MATH_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/types_c.h>

typedef std::vector<std::vector<cv::Point2f>> VecVecPoint2f;
typedef std::vector<cv::Point2f> VecPoint2f;
typedef std::pair<int, cv::Point2f> CornerIndex;
typedef std::vector<cv::Point3d> VecPoint3d;

// POINTS info
#define POINTS_NUM   80
#define ROI_ROW      300
#define ROI_COL      300

/// vector cv::Point to vector cv::Point2f
VecPoint2f Point2Point2f(const std::vector<cv::Point> VecPoint);

/// compute the angle between two angles
float GetAngleOfTwoVector(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Point2f& c);

/// sort near to far
VecPoint2f SortNearToFar(VecPoint2f& CornerPointList, const cv::Point2f& RefPoint, int n);

/// get nearest point
cv::Point2f GetNearestPoint(VecPoint2f& CornerPointList, const cv::Point2f& RefPoint);

/// get nearest point
cv::Point2f GetFarestPoint(VecPoint2f& CornerPointList, const cv::Point2f& RefPoint);

/// get cross point
cv::Point2f GetCrossPoint(const cv::Point2f& o1, const cv::Point2f& p1,
                            const cv::Point2f& o2, const cv::Point2f& p2);

/// set ROI region
std::vector<int> CodeRegion(cv::Mat& img, VecPoint2f& points);

#endif