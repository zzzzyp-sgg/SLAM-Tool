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

#include "frame.h"

void Frame::Display() {
    if (detected_qr_) {
        auto img = roi_img_;
        cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
        for (auto kp : keypoints_)
            cv::circle(img, kp, 2, cv::Scalar(0, 0, 255), cv::FILLED);
        for (auto corner : qr_corners_)
            cv::circle(img, corner, 5, cv::Scalar(255, 0, 0), cv::FILLED);

        printf("keypoints num: %ld\n", keypoints_.size());
        printf("QR corners num: %ld\n", qr_corners_.size());
        cv::imshow("detect features", img);
        cv::waitKey(0);        
    } else {
        printf("Have no QR code!\n");
    }
}

void Frame::DisplayOri() {
    if (detected_qr_) {
        auto img = src_img_;
        cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
        for (auto kp : keypoints_ori_)
            cv::circle(img, kp, 2, cv::Scalar(0, 0, 255), cv::FILLED);
        for (auto corner : qr_corners_ori_)
            cv::circle(img, corner, 5, cv::Scalar(255, 0, 0), cv::FILLED);

        printf("keypoints num: %ld\n", keypoints_ori_.size());
        printf("QR corners num: %ld\n", qr_corners_ori_.size());
        cv::imshow("detect features", img);
        cv::waitKey(0);        
    } else {
        printf("Have no QR code!\n");
    }
}
