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

#ifndef QR_CODE_H
#define QR_CODE_H

#include <vector>
#include <map>
// #include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>

typedef std::vector<std::vector<cv::Point2f>> VecVecPoint2f;
typedef std::vector<cv::Point2f> VecPoint2f;
typedef std::pair<int, cv::Point2f> CornerIndex;

class QRCode {
public:
    QRCode(const cv::Mat& image, int n = 1) {
        src_img_ = image;
        n_ = n;
    }

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

    /// locate the QR code
    void DetectCode();

    /// detect with quirc pre-detect
    void DetectQuirc();

    /// show result
    void Show(cv::Mat &im, VecPoint2f& corners);

    /// data interface
    std::map<int, cv::Point2f> GetCornerCenters() {
        return corner_centers_;
    }

    std::map<int, cv::Point2f> GetSubCorners() {
        return sub_corners_;
    }

    cv::Point2f GetQRCenter() {
        return center_;
    }

private:

    /// image processing
    void ImgProcessing();

    /// just filter and binaryzation
    void ImgPro(cv::Mat& img);

    /// filter profile
    std::vector<VecVecPoint2f> ProfileFilter();

    /// remove unused profile
    void ProfilePro(const std::vector<VecVecPoint2f>& qrPointList, std::vector<cv::RotatedRect>& RectList,
                    VecPoint2f& PointList, VecVecPoint2f& OutquadList);

    /// sort corner centers
    void CornerSort(std::vector<cv::RotatedRect>& RectList, VecPoint2f& PointList);

    /// precise location
    void Location(VecPoint2f& PointList, VecVecPoint2f& OutquadList);

    /// vector cv::Point to vector cv::Point2f
    VecPoint2f Point2Point2f(const std::vector<cv::Point> VecPoint);

private:
    /// input img
    cv::Mat src_img_;
    /// roi setting
    int n_;
    /// the image after processing
    cv::Mat pro_img_;
    /// three coner centers, to ensure the direction
    std::map<int, cv::Point2f> corner_centers_;
    /// 12 sub corners
    std::map<int, cv::Point2f> sub_corners_;
    /// qr_code center
    cv::Point2f center_;
    /// opencv detect
    VecPoint2f corners_;

    /// contours
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;
};

#endif