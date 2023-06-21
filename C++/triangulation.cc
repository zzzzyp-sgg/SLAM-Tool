#include <opencv2/opencv.hpp>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "Camera.h"

void featureDetection(cv::Mat img, std::vector<cv::Point2f> &kps_uv)
{
    // cv::Mat img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    int maxCorners = 240;
    cv::goodFeaturesToTrack(img, kps_uv, maxCorners, 0.01, 20);
}

void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d& d_u, const Eigen::Vector4d &distortionPara)
{
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

void liftProjective(cv::Point2f &kp_uv, cv::Point2f &kp, std::shared_ptr<Camera> cam)
{
    double mx_d, my_d;

    mx_d = kp_uv.x / cam->mFx - cam->mCx / cam->mFx;
    my_d = kp_uv.y / cam->mFy - cam->mCy / cam->mFy;

    Eigen::Vector4d distortionPara;
    distortionPara[0] = cam->mK1;
    distortionPara[1] = cam->mK2;
    distortionPara[2] = cam->mP1;
    distortionPara[3] = cam->mP2;

    int n = 8;
    Eigen::Vector2d d_u;
    distortion(Eigen::Vector2d(mx_d, my_d), d_u, distortionPara);
    double mx_u, my_u;
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);

    for (int i = 1; i < n; ++i)
    {
        distortion(Eigen::Vector2d(mx_u, my_u), d_u, distortionPara);
        mx_u = mx_d - d_u(0);
        my_u = my_d - d_u(1);
    }

    kp.x = mx_u;
    kp.y = my_u;
}

void reduceVector(std::vector<cv::Point2f> &kps, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(kps.size()); i++)
        if (status[i])
            kps[j++] = kps[i];
    kps.resize(j);
}

double distance(const cv::Point2f &kp1, const cv::Point2f &kp2)
{
    double dx = kp1.x - kp2.x;
    double dy = kp1.y - kp2.y;
    return sqrt(dx * dx + dy * dy);
}

void LK(const cv::Mat &img1, const cv::Mat &img2, std::vector<cv::Point2f> &kps1, std::vector<cv::Point2f> &kps2)
{
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(img1, img2, kps1, kps2, status, err, cv::Size(21, 21), 5);
    std::vector<uchar> reverse_status;
    std::vector<cv::Point2f> reverse_pts;
    cv::calcOpticalFlowPyrLK(img2, img1, kps2, reverse_pts, reverse_status, err, cv::Size(21, 21), 5);
    for (int i = 0; i < status.size(); i++) {
        if (status[i] && reverse_status[i] && distance(kps1[i], reverse_pts[i]) < 0.5) {
            status[i] = 1;
        } else {
            status[i] = 0;
        }
    }

    reduceVector(kps1, status);
    reduceVector(kps2, status);
}

void poseEstimation(const std::vector<cv::Point2f> &kp1, const std::vector<cv::Point2f> &kp2, cv::Mat &R, cv::Mat &t, std::shared_ptr<Camera> cam)
{
    // 相机内参， TUM, Freiburg2
    cv::Mat K = (cv::Mat_<double>(3, 3) << cam->mFx, 0, cam->mCx, 0, cam->mFy, cam->mCy, 0, 0, 1);

    cv::Point2d principalPoint(cam->mCx, cam->mCy);
    double focalLength = (cam->mFx + cam->mFy) / 2;
    cv::Mat E = cv::findEssentialMat(kp1, kp2, focalLength, principalPoint);

    cv::recoverPose(E, kp1, kp2, R, t, focalLength, principalPoint);   
}

void triangulation(const std::vector<cv::Point2f> &kp1, const std::vector<cv::Point2f> &kp2, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &points)
{
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

int main(int argc, char **argv)
{
    if (argc != 3) {
        std::cout << "Input: config_file, image_path!" << std::endl;
        exit(0);
    }

    std::string config = argv[1];
    std::string imgPath = argv[2];
    std::string img1File = imgPath + "/image_00_dw/data/0000000000.png";
    cv::Mat img1 = cv::imread(img1File, CV_LOAD_IMAGE_GRAYSCALE);
    std::string img2File = imgPath + "/image_00_dw/data/0000000001.png";
    cv::Mat img2 = cv::imread(img2File, CV_LOAD_IMAGE_GRAYSCALE);
    std::string img1RightFile = imgPath + "/image_01_dw/data/0000000000.png";
    cv::Mat img1Right = cv::imread(img1RightFile, CV_LOAD_IMAGE_GRAYSCALE);
    std::shared_ptr<Camera> cam(new Camera());

    cam->readParameters(config);

    std::vector<cv::Point2f> kps1_uv, kps2_uv, kps1, kps2;
    featureDetection(img1, kps1_uv);

    if (0)
    {
        kps2_uv.resize(kps1_uv.size());
        LK(img1, img2, kps1_uv, kps2_uv);
        kps1.resize(kps1_uv.size());
        kps2.resize(kps2_uv.size());
        for (int i = 0; i < kps1_uv.size(); i++) {
            liftProjective(kps1_uv[i], kps1[i], cam);
            liftProjective(kps2_uv[i], kps2[i], cam);
        }
    }
    
    std::vector<cv::Point2f> kps1_r_uv, kps1_r;
    kps1_r_uv.resize(kps1_uv.size());
    LK(img1, img1Right, kps1_uv, kps1_r_uv);
    kps1.resize(kps1_uv.size());
    kps1_r.resize(kps1_r_uv.size());
    for (int i = 0; i < kps1.size(); i++)
    {
        liftProjective(kps1_uv[i], kps1[i], cam);
        liftProjective(kps1_r_uv[i], kps1_r[i], cam);
    }

    std::vector<cv::Point3d> points;
    cv::Mat R, t;
    poseEstimation(kps1_uv, kps1_r_uv, R, t, cam);
    triangulation(kps1, kps1_r, R, t, points);
    for (int i = 1; i < points.size() + 1; i++)
    {
        std::cout << points[i].z << " ";
        if ((i % 5) == 0)
            std::cout << std::endl;
    }
    
    return 0;
}