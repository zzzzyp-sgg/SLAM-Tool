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

#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <memory>
#include "../qr_code/qr_math.h"
#include "../qr_code/qr_code.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class Frame {

public:
    Frame(const cv::Mat &img, const int &scale) : src_img_(img), scale_(scale) {
        if (scale == 0)
            return;

        QRCode qr(src_img_, scale_);
        qr_ = std::make_shared<QRCode>(qr);

        // detect qr code
        detected_qr_ = qr_->DetectQuirc();
        if (detected_qr_) {
            // detect keypoints
            qr_->DetectFeature();

            // get pro_img and keypoints
            keypoints_  = qr_->GetKeypoints();
            roi_img_    = qr_->GetProImg();
            qr_corners_ = qr_->GetCorners();
            delta_      = qr_->GetDeltaCoor();
            ToOriCoor();          
        } else {
            // printf("Cannot detect QR code, please move the device around!\n");
        }

    };

    /// show detect result
    void Display();
    void DisplayOri();

    /// whether the frame is empty
    bool Empty() {
        return src_img_.empty();
    }

    cv::Mat GetSrcImg() {
        return src_img_;
    }

    cv::Mat GetROI() {
        return roi_img_;
    }

    VecPoint2f GetKeyPoints() {
        return keypoints_;
    }

    VecPoint2f GetCorners() {
        return qr_corners_;
    }

    VecPoint2f GetKeyPointsOri() {
        return keypoints_ori_;
    }

    VecPoint2f GetCornersOri() {
        return qr_corners_ori_;
    }

    bool DetectQR() {
        return detected_qr_;
    }

private:
    void ToOriCoor() {
        keypoints_ori_ = keypoints_;
        qr_corners_ori_ = qr_corners_;
        for (auto &kp : keypoints_ori_) {
            kp.x += delta_.at(0);
            kp.y += delta_.at(1);
        }
        for (auto &c : qr_corners_ori_) {
            c.x += delta_.at(0);
            c.y += delta_.at(1);
        }
    }

private:
    /// image
    cv::Mat src_img_, roi_img_;
    /// keypoints
    VecPoint2f keypoints_, keypoints_ori_;
    /// qr code detector
    std::shared_ptr<QRCode> qr_;
    /// qr corners
    VecPoint2f qr_corners_, qr_corners_ori_;
    /// scale
    int scale_;
    /// detect qr code
    bool detected_qr_;
    /// coor delta
    std::vector<int> delta_;
};

#endif