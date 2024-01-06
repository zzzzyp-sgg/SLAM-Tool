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

#include "estimator.h"
#include <chrono>

Estimator::Estimator(std::shared_ptr<Camera> cam) {
    cam_ = cam;
    first_frame_ = true;
    has_frame_ = false;
    last_R_ = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    last_p_ = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    // #define CV_32F   5
    // cv::Mat mat = cv::Mat::zeros(100, 100, 5);
    // last_f_ = std::make_shared<Frame>(Frame(mat, 0));
}

void Estimator::AddFrame(std::shared_ptr<Frame> f)
{
    // here is the first frame
    if (!has_frame_ && !f->DetectQR()) {
        printf("Cannot detect QR code, please move the device around!\n");
        last_f_ = f;
        last_corners_ = last_f_->GetCornersOri();
        last_kps_ = last_f_->GetKeyPointsOri();
        first_frame_ = true;
        // has_frame_ = true;
    } else {
        first_frame_ = false;
        has_frame_ = true;
    }
    
    // current is always updated
    cur_f_ = f;
    cur_corners_ = cur_f_->GetCornersOri();
    cur_kps_ = cur_f_->GetKeyPointsOri();
}

bool Estimator::EstProcssing() {
    VecPoint2f last_kps, cur_kps, last_kps_un, cur_kps_un;
    // the last 4 members of this vector will be qr code corners
    last_kps = last_kps_;
    cur_kps = cur_kps_;
    VecPoint3d points;
    cv::Mat R, t;
    if (!first_frame_) {
        if (!last_f_->DetectQR() || !cur_f_->DetectQR()) {
            if (!last_f_->DetectQR()) {
                printf("Init begins!\n");
                last_f_ = cur_f_;
                last_kps_ = cur_kps_;
                last_corners_ = cur_corners_;
                return false;                
            }

            if (!cur_f_->DetectQR()) {
                printf("Move away! Unuse this frame.\n");
                ReInit();
                return false;                
            }
        }

        auto t1 = std::chrono::steady_clock::now();
        // make sure the vector length is same
        if (last_kps_.size() != cur_kps_.size())
            cur_kps_.resize(last_kps_.size());

        // LK(last_f_->GetROI(), cur_f_->GetROI(), last_kps, cur_kps);
        LK(last_f_->GetSrcImg(), cur_f_->GetSrcImg(), last_kps, cur_kps);
        last_kps.insert(last_kps.end(), last_corners_.begin(), last_corners_.end());
        cur_kps.insert(cur_kps.end(), cur_corners_.begin(), cur_corners_.end());
        // int i = 1;
        // for (auto lc : last_corners_)
        //     printf("last corner%d : %f, %f\n", i++, lc.x, lc.y);
        // i = 1;
        // for (auto cc : cur_corners_)
        //     printf("cur corner%d : %f, %f\n", i++, cc.x, cc.y);
        LiftProjective(last_kps, last_kps_un);
        LiftProjective(cur_kps, cur_kps_un);
        VecPoint2f cur_corners_un, last_corners_un;
        LiftProjective(last_corners_, last_corners_un);
        LiftProjective(cur_corners_, cur_corners_un);
        // LiftProjective(last_corners_, last_kps_un, cam_);
        // LiftProjective(cur_corners_, cur_kps_un, cam_);

        PoseEstimation(last_kps, cur_kps, R, t);
        Triangulation(last_kps_un, cur_kps_un, R, t, points);

        int out_num = 1;
        VecPoint3d corner_3d;
        // printf("/ -------- triangulation result --------/\n");
        std::for_each(points.rbegin(), points.rend(), [&out_num, &corner_3d](const cv::Point3d &p){
            // printf("x : %f, y: %f, z: %f\n", p.x, p.y, p.z);
            if (out_num <= 4) {
                // printf("x : %f, y: %f, z: %f\n", p.x, p.y, p.z);
                corner_3d.push_back(p);
            }
            out_num++;
        });
        // printf("/ --------------------------------------/\n");
        // corners_3d_ = corner_3d;

        double scale = ComputeScale(corner_3d);
        AlignScale(points, scale);
        out_num = 0;
        std::for_each(points.rbegin(), points.rend(), [&out_num, &corner_3d](const cv::Point3d &p){
            // printf("x : %f, y: %f, z: %f\n", p.x, p.y, p.z);
            if (out_num < 4) {
                // printf("3D corner-points x : %f, y: %f, z: %f\n", p.x, p.y, p.z);
                corner_3d.at(out_num++) = p;
            }
        });
        // printf("/ --------------------------------------/\n");
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        cv::cv2eigen(last_R_, R);
        cv::cv2eigen(last_p_, t);
        auto tmp = corner_3d;
        CoorToWorld(R, t, tmp);
        std::reverse(tmp.begin(), tmp.end());
        SolvePosePnP(R, t, cur_corners_un, tmp);
        // SolvePosePnP(R, t, last_corners_un, tmp);
        printf("The translation in the Z direction is: %f\n", 
               t.z());
        // printf("The error in the Z direction is: %f\n", 
        //        t.z());

        cv::eigen2cv(R, cur_R_);
        cv::eigen2cv(t, cur_p_);
        auto t2 = std::chrono::steady_clock::now();
        auto tt = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        printf("Every estimation costs: %ldms\n", tt.count());

        // update
        last_f_ = cur_f_;
        last_kps_ = cur_kps_;
        last_corners_ = cur_corners_;
        last_R_ = cur_R_;
        last_p_ = cur_p_;
    }
    return true;
}

#if 0
/// TODO 目前还不能用，还没有找BUG
void Estimator::EstProcssingH() {
    // make sure the vector length is same
    if (last_kps_.size() != cur_kps_.size())
        cur_kps_.resize(last_kps_.size());

    VecPoint2f last_kps, cur_kps;
    // the last 4 members of this vector will be qr code corners
    last_kps = last_kps_;
    cur_kps = cur_kps_;
    cv::Mat K = (cv::Mat_<double>(3, 3) << cam_->mFx, 0, cam_->mCx, 0, cam_->mFy, cam_->mCy, 0, 0, 1);
    VecPoint3d points;
    cv::Mat R, t;
    float score;
    cv::Mat H21;
    VecBool inlierH, vbTriangulated;
    if (!first_frame_) {
        // LK(last_f_->GetROI(), cur_f_->GetROI(), last_kps, cur_kps);
        LK(last_f_->GetSrcImg(), cur_f_->GetSrcImg(), last_kps, cur_kps);
        last_kps.insert(last_kps.end(), last_corners_.begin(), last_corners_.end());
        cur_kps.insert(cur_kps.end(), cur_corners_.begin(), cur_corners_.end());
        int i = 1;
        for (auto lc : last_corners_)
            printf("last corner%d : %f, %f\n", i++, lc.x, lc.y);
        i = 1;
        for (auto cc : cur_corners_)
            printf("cur corner%d : %f, %f\n", i++, cc.x, cc.y);
        
        FindH(last_kps, cur_kps, score, H21, inlierH);
        ReconstructH(inlierH, H21, K, last_kps, cur_kps, R, t, points, vbTriangulated, 1.0, 50);
        int out_num = 1;
        printf("/ -------- triangulation result --------/\n");
        std::for_each(points.rbegin(), points.rend(), [&out_num](const cv::Point3d &p){
            // printf("x : %f, y: %f, z: %f\n", p.x, p.y, p.z);
            if (out_num <= 4) printf("x : %f, y: %f, z: %f\n", p.x, p.y, p.z);
            out_num++;
        });
        printf("/ --------------------------------------/\n");
    }
}

#endif

void Estimator::UnDistortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u, 
                             const Eigen::Vector4d &distortionPara) {

    double k1 = distortionPara[0], k2 = distortionPara[1];
    double p1 = distortionPara[2], p2 = distortionPara[3];

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void Estimator::LiftProjective(const VecPoint2f &kp_uvs, VecPoint2f &kp) {
    double mx_d, my_d;
    for (auto kp_uv : kp_uvs) {
        mx_d = kp_uv.x / cam_->mFx - cam_->mCx / cam_->mFx;
        my_d = kp_uv.y / cam_->mFy - cam_->mCy / cam_->mFy;

        Eigen::Vector4d distortionPara;
        distortionPara[0] = cam_->mK1;
        distortionPara[1] = cam_->mK2;
        distortionPara[2] = cam_->mP1;
        distortionPara[3] = cam_->mP2;

        int n = 8;
        Eigen::Vector2d d_u;
        UnDistortion(Eigen::Vector2d(mx_d, my_d), d_u, distortionPara);
        double mx_u, my_u;
        mx_u = mx_d - d_u(0);
        my_u = my_d - d_u(1);

        for (int i = 1; i < n; ++i)
        {
            UnDistortion(Eigen::Vector2d(mx_u, my_u), d_u, distortionPara);
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);
        }

        kp.push_back(cv::Point2f(mx_u, my_u));       
    }

}

double Estimator::Distance(const cv::Point2f &kp1, const cv::Point2f &kp2) {
    double dx = kp1.x - kp2.x;
    double dy = kp1.y - kp2.y;
    return sqrt(dx * dx + dy * dy);
}

double Estimator::Distance(const cv::Point3d &kp1, const cv::Point3d &kp2) {
    double dx = kp1.x - kp2.x;
    double dy = kp1.y - kp2.y;
    double dz = kp1.z - kp2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void Estimator::LK(const cv::Mat &img1, const cv::Mat &img2, VecPoint2f &kps1, VecPoint2f &kps2) {
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(img1, img2, kps1, kps2, status, err, cv::Size(21, 21), 5);
    std::vector<uchar> reverse_status;
    VecPoint2f reverse_pts;
    cv::calcOpticalFlowPyrLK(img2, img1, kps2, reverse_pts, reverse_status, err, cv::Size(21, 21), 5);
    for (int i = 0; i < status.size(); i++) {
        if (status[i] && reverse_status[i] && Distance(kps1[i], reverse_pts[i]) < 0.5) {
            status[i] = 1;
        } else {
            status[i] = 0;
        }
    }

    ReduceVector(kps1, status);
    ReduceVector(kps2, status);
}

void Estimator::PoseEstimation(const VecPoint2f &kp1, const VecPoint2f &kp2, 
                               cv::Mat &R, cv::Mat &t) {

    // camera info
    cv::Mat K = (cv::Mat_<double>(3, 3) << cam_->mFx, 0, cam_->mCx, 0, cam_->mFy, cam_->mCy, 0, 0, 1);

    cv::Point2d principalPoint(cam_->mCx, cam_->mCy);
    double focalLength = (cam_->mFx + cam_->mFy) / 2;
    cv::Mat E = cv::findEssentialMat(kp1, kp2, focalLength, principalPoint);

    cv::recoverPose(E, kp1, kp2, R, t, focalLength, principalPoint);
}

void Estimator::Triangulation(const VecPoint2f &kp1, const VecPoint2f &kp2, 
                              cv::Mat &R, cv::Mat &t, VecPoint3d &points) {
    cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    cv::Mat pts4d;
    cv::triangulatePoints(T1, T2, kp1, kp2, pts4d);

    for (int i = 0; i < pts4d.cols; i++)
    {
        cv::Mat x = pts4d.col(i);
        x /= x.at<float>(3, 0);  // 归一化
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}

double Estimator::ComputeScale(const VecPoint3d &p3d) {
    auto d1 = Distance(p3d.at(0), p3d.at(1));
    auto d2 = Distance(p3d.at(1), p3d.at(2));
    auto d3 = Distance(p3d.at(2), p3d.at(3));
    auto d4 = Distance(p3d.at(3), p3d.at(1));

    return 0.20 / (d1 + d2+ d3+ d4);
}

void Estimator::AlignScale(VecPoint3d &p3d, const double &scale) {
    // TODO 后边可以加一个平面拟合？把那些明显不好的点给剔除喽
    for (auto &p : p3d) {
        p *= scale;
    }
}

bool Estimator::SolvePosePnP(Eigen::Matrix3d &R, Eigen::Vector3d &p, VecPoint2f &pts2d, VecPoint3d &pts3d)
{

    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;
 
    //坐标系之间的转换：w_T_cam ---> cam_T_w，注意和点之间转换的区别
    R_initial = R.inverse();
    P_initial = -(R_initial * p);
 
   
    if (int(pts2d.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);//cv::Mat 与 Eigen 转换
    cv::Rodrigues(tmp_r, rvec);    //罗德里格斯公式
    cv::eigen2cv(P_initial, t);     //cv::Mat 与 Eigen 转换
    
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3d, pts2d, K, D, rvec, t, 1);//用的是相似三角形方法！
   
 
    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    
    cv::Rodrigues(rvec, r);
    
     
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
 
    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    p = R * (-T_pnp);
 
    return true;
}

void Estimator::CoorToWorld(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, VecPoint3d &points)
{
    Eigen::Vector3d point;
    for (auto &p : points) {
        point << p.x, p.y, p.z;
        point = R.inverse() * point + t;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
    }
}

void Estimator::ReInit()
{
    cv::Mat img = cv::Mat::zeros(3, 3, 5);
    // current states
    cur_f_ = std::make_shared<Frame>(Frame(img, 0));
    cur_corners_.clear();
    cur_kps_.clear();
    cur_p_ = cv::Mat::zeros(last_p_.size(), 0);
    cur_R_ = cv::Mat::zeros(last_R_.size(), 0);

    // last states
    first_frame_ = true;
    has_frame_ = false;
    last_R_ = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    last_p_ = (cv::Mat_<double>(3, 1) << 0, 0, 0);
}
