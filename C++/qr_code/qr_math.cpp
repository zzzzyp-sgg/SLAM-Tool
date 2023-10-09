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

#include "qr_math.h"

/// vector cv::Point to vector cv::Point2f
VecPoint2f Point2Point2f(const std::vector<cv::Point> VecPoint) {
    VecPoint2f vec_point2f(VecPoint.size());
    for (int i = 0; i < VecPoint.size(); i++) {
        vec_point2f.at(i) = cv::Point2f(VecPoint.at(i));
    }

    return vec_point2f;
}

/// compute the angle between two angles
float GetAngleOfTwoVector(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Point2f& c) {
    float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
    if (theta > CV_PI)
        theta -= 2 * CV_PI;
    if (theta < -CV_PI)
        theta += 2 * CV_PI;
    theta = theta * 180.0 / CV_PI;

    return theta;
}

/// sort near to far
VecPoint2f SortNearToFar(VecPoint2f& CornerPointList, const cv::Point2f& RefPoint, int n) {

    sort(CornerPointList.begin(), CornerPointList.end(), 
         [RefPoint](const cv::Point2f& pt1, const cv::Point2f& pt2){

        float distance_1 = cv::norm(pt1 - RefPoint);
        float distance_2 = cv::norm(pt2 - RefPoint);
        return distance_1 < distance_2;
    });

    VecPoint2f NearCornerList;
    for (int i = 0; i < n; i++) {
        NearCornerList.push_back(CornerPointList.at(i));
    }

    return NearCornerList;
}

/// get nearest point
cv::Point2f GetNearestPoint(VecPoint2f& CornerPointList, const cv::Point2f& RefPoint) {
    SortNearToFar(CornerPointList, RefPoint, 4);
    return CornerPointList.at(0);
}

/// get nearest point
cv::Point2f GetFarestPoint(VecPoint2f& CornerPointList, const cv::Point2f& RefPoint) {
    SortNearToFar(CornerPointList, RefPoint, 4);
    return CornerPointList.at(CornerPointList.size() - 1);
}

/// get cross point
cv::Point2f GetCrossPoint(const cv::Point2f& o1, const cv::Point2f& p1,
                            const cv::Point2f& o2, const cv::Point2f& p2) {

    cv::Point2f x = o2 - o1;
    cv::Point2f d1 = p1 - o1;
    cv::Point2f d2 = p2 - o2;

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        printf("Please check the points!\n");

    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
    auto r = o1 + d1 * t1;
    return r;
}

std::vector<int> CodeRegion(cv::Mat &img, VecPoint2f &points) {
    int col_max = 0, row_max = 0, col_min = 10000, row_min = 10000;
    // min and max coor value of corners
    for (int i = 0; i < points.size(); i++) {
        if (points.at(i).x > col_max)
            col_max = points.at(i).x;
        if (points.at(i).x < col_min)
            col_min = points.at(i).x;
        if (points.at(i).y > row_max)
            row_max = points.at(i).y;
        if (points.at(i).y < row_min)
            row_min = points.at(i).y;
    }
    int col_left  = int((ROI_COL - (col_max - col_min)) / 2);
    int col_right = int((ROI_COL - (col_max - col_min) - col_left));
    int row_up    = int((ROI_ROW - (row_max - row_min)) / 2);
    int row_down  = int((ROI_ROW - (row_max - row_min) - row_up));

    col_min = ((col_min - col_left) < 0) ? 0 : col_min - col_left;
    col_max = ((col_max + col_right) > img.cols) ? img.cols : col_max + col_right;
    row_min = ((row_min - row_up) < 0) ? 0 : row_min - row_up;
    row_max = ((row_max + row_down) > img.rows) ? img.rows : row_max + row_down;

    std::vector<int> delta{col_min, row_min};

    // set ROI
    auto img_tmp = img(cv::Range(row_min, row_max), cv::Range(col_min, col_max));
    img = img_tmp;
    for (auto &kp : points) {
        kp.x -= col_min;
        kp.y -= row_min;
    }

    return delta;
}
