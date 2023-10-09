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

#include "qr_math.h"

class QRCode {
public:
    QRCode(const cv::Mat &image, int n = 1) {
        src_img_ = image;
        n_ = n;
    }

    /// locate the QR code without quirc
    void DetectCode();

    /// detect with quirc pre-detect
    bool DetectQuirc();

    /// show result
    void ShowDetect(cv::Mat &im, VecPoint2f &corners);

    /// detect feature points
    void DetectFeature();

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

    cv::Mat GetProImg() {
        return pro_img_;
    }

    VecPoint2f GetKeypoints() {
        return keyPoints_;
    }

    VecPoint2f GetCorners() {
        return corners_;
    }

    std::vector<int> GetDeltaCoor() {
        std::vector<int> delta{col_delta_, row_delta_};
        return delta;
    }

private:

    /// image processing
    void ImgProcessing();

    /// just filter and binaryzation
    void ImgPro(cv::Mat &img);

    /// filter profile
    std::vector<VecVecPoint2f> ProfileFilter();

    /// remove unused profile
    void ProfilePro(const std::vector<VecVecPoint2f> &qrPointList, std::vector<cv::RotatedRect> &RectList,
                    VecPoint2f &PointList, VecVecPoint2f &OutquadList);

    /// sort corner centers
    void CornerSort(std::vector<cv::RotatedRect> &RectList, VecPoint2f &PointList);

    /// precise location
    void Location(VecPoint2f &PointList, VecVecPoint2f &OutquadList);

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
    /// feature points
    VecPoint2f keyPoints_;
    /// recover the coor to origin img
    int col_delta_, row_delta_;

    /// contours
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;
};

#endif