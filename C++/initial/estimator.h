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

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "frame.h"
#include "Camera.h"
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

class Estimator {

public:
    /// constructor 
    Estimator(std::shared_ptr<Camera> cam);

    /// add frame
    void AddFrame(std::shared_ptr<Frame> f);

    /// estimate process
    bool EstProcssing();

    /// estimate process
    /// void EstProcssingH();

private:
    /// dedistortion of pixels
    void UnDistortion(const Eigen::Vector2d &p_u, Eigen::Vector2d& d_u, const Eigen::Vector4d &distortionPara);

    /// convert pixel coordinates to normalized coordinates without distortion
    void LiftProjective(const VecPoint2f &kp_uvs, VecPoint2f &kp);

    /// resize the vector according to status vector
    // void ReduceVector(VecPoint2f &kps, std::vector<uchar> status);
    template<typename VecType>
    void ReduceVector(VecType &v, std::vector<uchar> status) {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    /// calcilate the distance between two points
    double Distance(const cv::Point2f &kp1, const cv::Point2f &kp2);
    double Distance(const cv::Point3d &kp1, const cv::Point3d &kp2);

    /// LK tracker
    void LK(const cv::Mat &img1, const cv::Mat &img2, VecPoint2f &kps1, VecPoint2f &kps2);

    /// pose estimation, easy method by opencv
    void PoseEstimation(const VecPoint2f &kp1, const VecPoint2f &kp2, 
                        cv::Mat &R, cv::Mat &t);

    /// triangulate points, easy method by opencv
    void Triangulation(const VecPoint2f &kp1, const VecPoint2f &kp2, 
                       cv::Mat &R, cv::Mat &t, VecPoint3d &points);

    /// compute real scale
    double ComputeScale(const VecPoint3d &p3d);

    /// align the scale
    void AlignScale(VecPoint3d &points, const double &scale);

    /// solve pose PnP
    bool SolvePosePnP(Eigen::Matrix3d &R, Eigen::Vector3d &p, VecPoint2f &pts2d, VecPoint3d &pts3d);

    /// trans 3d points to world coor
    void CoorToWorld(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, VecPoint3d &points);

    /// reinit
    void ReInit();

private:
    /// frame
    std::shared_ptr<Frame> last_f_, cur_f_;
    /// qr code corners
    VecPoint2f last_corners_, cur_corners_;
    /// keypoints
    VecPoint2f last_kps_, cur_kps_;
    /// roi img
    // cv::Mat last_img_, cur_img_;

    bool first_frame_;
    bool has_frame_;
    bool is_ok_;

    /// camera
    std::shared_ptr<Camera> cam_;
    /// corner points 3D
    VecPoint3d corners_3d_;
    /// Pose
    cv::Mat last_R_, last_p_;
    cv::Mat cur_R_, cur_p_;
};

#endif