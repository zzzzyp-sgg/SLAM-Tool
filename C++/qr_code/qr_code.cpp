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

#include "qr_code.h"

float QRCode::GetAngleOfTwoVector(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Point2f& c) {

    float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
    if (theta > CV_PI)
        theta -= 2 * CV_PI;
    if (theta < -CV_PI)
        theta += 2 * CV_PI;
    theta = theta * 180.0 / CV_PI;

    return theta;
}


VecPoint2f QRCode::SortNearToFar(VecPoint2f& CornerPointList, 
                                 const cv::Point2f& RefPoint, int n) {

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

cv::Point2f QRCode::GetNearestPoint(VecPoint2f &CornerPointList, const cv::Point2f &RefPoint) {
    SortNearToFar(CornerPointList, RefPoint, 4);
    return CornerPointList.at(0);
}

cv::Point2f QRCode::GetFarestPoint(VecPoint2f &CornerPointList, const cv::Point2f &RefPoint) {
    SortNearToFar(CornerPointList, RefPoint, 4);
    return CornerPointList.at(CornerPointList.size() - 1);
}

cv::Point2f QRCode::GetCrossPoint(const cv::Point2f &o1, const cv::Point2f &p1, 
                                  const cv::Point2f &o2, const cv::Point2f &p2)
{
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

void QRCode::DetectCode() {
    // pre-process the img
    ImgProcessing();

    // detect the profile
    std::vector<VecVecPoint2f> qrPointList;
    qrPointList = ProfileFilter();

    // process
    std::vector<cv::RotatedRect> RectList;
    VecPoint2f PointList;
    VecVecPoint2f OutquadList;
    ProfilePro(qrPointList, RectList, PointList, OutquadList);

    // sort corner centers
    CornerSort(RectList, PointList);

    // locate
    Location(PointList, OutquadList);
}

void QRCode::DetectQuirc() {
    int row = src_img_.rows / n_;
    int col = src_img_.cols / n_;
    auto tmp_img = (n_ == 1) ? src_img_ : src_img_(cv::Range(row, (n_ - 1) * row),
                                                   cv::Range(col, (n_ - 1) * col));

    ImgPro(tmp_img);
    cv::QRCodeDetector qrDecoder = cv::QRCodeDetector();
    qrDecoder.detect(tmp_img, corners_);

    Show(tmp_img, corners_);

    cv::imshow("detect result", tmp_img);
    cv::waitKey(0);
}

void QRCode::Show(cv::Mat &im, VecPoint2f& corners) {
    int n = corners.size();
	for (int i = 0; i < n; i++)
	{
		cv::circle(im, corners.at(i), 10, (255, 0, 0));
	}

	// cv::imshow("Detect result", im);
    // cv::waitKey(0);
}



void QRCode::ImgProcessing() {

    cv::Mat tmp_img;
    if (n_ != 1) {
        int row_0 = src_img_.rows / n_;
        int col_0 = src_img_.cols / n_;
        tmp_img = src_img_(cv::Range(row_0, (n_ - 1) * row_0), cv::Range(col_0, col_0 * (n_ - 1)));
    } else {
        tmp_img = src_img_;
    }
    
    // gauss filter
    cv::GaussianBlur(tmp_img, tmp_img, cv::Size(3, 3), 2, 2, cv::BORDER_DEFAULT);
    cv::imshow("gauss filter", tmp_img);
    cv::waitKey(0);

    // binaryzation
    // cv::threshold(tmp_img, tmp_img, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::adaptiveThreshold(tmp_img, tmp_img, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 2);
    cv::imshow("binaryzation", tmp_img);
    // cv::imwrite("./res/10_thre_100.jpg", tmp_img);
    cv::waitKey(0);

    // morphological filtering
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(tmp_img, tmp_img, cv::MORPH_OPEN, element);
    cv::morphologyEx(tmp_img, tmp_img, cv::MORPH_CLOSE, element);
    cv::imshow("morphological filtering", tmp_img);
    cv::waitKey(0);

    // canny
    // TODO 不做canny试一下
    cv::Canny(tmp_img, tmp_img, 1, 3, 7, true);
    cv::imshow("canny", tmp_img);
    cv::waitKey(0);
    // find contours
    // std::vector<std::vector<cv::Point2f>> contours;
    // std::vector<cv::Vec4i> hierarchy;
    cv::findContours(tmp_img, contours_, hierarchy_, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point());

    cv::Mat Contours=cv::Mat::zeros(tmp_img.size(),CV_8UC1);  
    //绘制 //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
    for(int i=0;i<contours_.size();i++)
        for(int j = 0; j < contours_[i].size(); j++){
            cv::Point P = cv::Point(contours_[i][j].x, contours_[i][j].y);
            Contours.at<uchar>(P)=255;
        }  
    cv::imshow("5.findContours轮廓提取", Contours); //轮廓
    cv::waitKey(0);
    pro_img_ = tmp_img.clone();
}

void QRCode::ImgPro(cv::Mat& img) {
    // gauss filter
    cv::GaussianBlur(img, img, cv::Size(1, 1), 2, 2, cv::BORDER_DEFAULT);

    // normalize
    cv::normalize(img, img, 0, 255, cv::NORM_MINMAX);
}

std::vector<VecVecPoint2f> QRCode::ProfileFilter() {

    cv::Mat color_mat;
    VecVecPoint2f qrPoint;
    std::vector<VecVecPoint2f> qrPointList;
    cv::cvtColor(pro_img_, color_mat, cv::COLOR_GRAY2BGR);
    int Parindex[3] = {-1, -1, -1};

    for (int i = 0; i < contours_.size(); i++) {
        // parent hierarchy
        if (hierarchy_[i][3] != -1 && hierarchy_[i][2] == -1) {
            Parindex[0] = hierarchy_[i][3];
            if (hierarchy_[(Parindex[0])][3] != -1){
                Parindex[1] = hierarchy_[(Parindex[0])][3];
                if (hierarchy_[(Parindex[1])][3] != -1){
                    Parindex[2] = hierarchy_[(Parindex[1])][3];
                    if (hierarchy_[(Parindex[2])][3] != -1){
                        if(!(i - 1 == Parindex[0] 
                            && Parindex[0]-1 == Parindex[1]&&Parindex[1] - 1 == Parindex[2]))
                            continue; // individual
                        qrPoint.push_back(Point2Point2f(contours_[i]));
                        qrPoint.push_back(Point2Point2f(contours_[i-2]));
                        qrPoint.push_back(Point2Point2f(contours_[i-4]));
                        qrPointList.push_back(qrPoint);
                        qrPoint.clear();
                    }
                }
            }
        }
    }

    return qrPointList;
}

/**
 * @param[in]       qrPointList
 * @param[inout]    RecList         The outermost rectangle of the corner element
 * @param[inout]    PointList       Corner center
 * @param[inout]    OutquadList     The outermost layer fits the quadrilateral corner points
*/
void QRCode::ProfilePro(const std::vector<VecVecPoint2f>& qrPointList, std::vector<cv::RotatedRect>& RectList,
                        VecPoint2f& PointList, VecVecPoint2f& OutquadList) {

    std::vector<bool> qrPointListEnable(qrPointList.size());

    for (int L = 0; L < qrPointList.size(); L++) {
        auto qrPoint = qrPointList.at(L);
        VecVecPoint2f contours_poly(qrPoint.size());
        // minimum external rectangle
        std::vector<cv::RotatedRect> minRect(qrPoint.size());
        VecPoint2f rect_center(qrPoint.size());
        for (int i = 0; i < qrPoint.size(); i++){
            cv::approxPolyDP(cv::Mat(qrPoint[i]), contours_poly[i], 5, true);
            minRect[i] = cv::minAreaRect(cv::Mat(qrPoint[i]));  // get minimum external rectangle
            rect_center[i]=minRect[i].center;                   // get rec center
        }

        // filter by concentricity
        for (int i = 0; i < minRect.size()-1; i++){
            auto P1 = cv::Point2f(rect_center[i].x, rect_center[i].y);
            auto P2 = cv::Point2f(rect_center[i + 1].x, rect_center[i + 1].y);
            float ConcenError_Set = (minRect[i].size.width+minRect[i].size.height) / 12;
            if(sqrt(pow(P1.x-P2.x, 2) + pow(P1.y-P2.y, 2)) > ConcenError_Set) {
                qrPointListEnable[L]=false;
                break; 
            }
            else
                qrPointListEnable[L]=true;
        }
        if (!qrPointListEnable[L]) 
            continue;
        
        // filter according to the ratio of three layers
        for (int i = 0; i < minRect.size()-1; i++) {
            float circum1=(minRect[i].size.width + minRect[i].size.height)*2;
            float circum2=(minRect[i+1].size.width + minRect[i+1].size.height)*2;
            if( circum1 / circum2 >= 0.5 && circum1 / circum2 <= 0.8 )
                qrPointListEnable[L]=true;
            else{
                qrPointListEnable[L]=false;
                break; 
            }
        }
        if(!qrPointListEnable[L])
            continue;

        // whether the circum is to small
        for (int i = 0; i < minRect.size(); i++){
            float circum = (minRect[i].size.width + minRect[i].size.height) * 2;
            float circum_Set = 20;
            if( circum >= circum_Set )
                qrPointListEnable[L]=true;
            else{
                qrPointListEnable[L]=false;
                break; 
            }
        }
        if(!qrPointListEnable[L])
            continue;

        // store result
        for (int i = 0; i < qrPoint.size(); i++){
            cv::Point2f rect_points[4];
            minRect[i].points(rect_points);
            if(i == 2)
                RectList.push_back(minRect[i]);        // RectList
            bool exsit=false;
            auto P = cv::Point2f(rect_center[i].x, rect_center[i].y);
            for(int j = 0; j < PointList.size();j++){
                if(fabs(PointList.at(j).x - P.x) < 10 && fabs(PointList.at(j).y - P.y) < 10){
                    exsit=true; break; }
            }
            if(!exsit||PointList.size()==0)
                PointList.push_back(P);                     // pointList
            if(i==2)
                OutquadList.push_back(contours_poly[i]);    // OutquadList
        }
    }

    // final filter
    if (RectList.size() > 3) {
        std::vector<float> RectSizeErrorList; // size error
        for(int i = 0; i < RectList.size(); i++){
            float RectSizeError = 0;
            float RectSize1 = (RectList.at(i).size.width + RectList.at(i).size.height) * 2;
            for(int j = 0; j < RectList.size(); j++ && j != i){
                float RectSize2 = (RectList.at(j).size.width + RectList.at(j).size.height) * 2;
                float Error = fabs(RectSize1 - RectSize2);
                RectSizeError += Error;
            }
            RectSizeErrorList.push_back(RectSizeError);
        }

        std::vector<float> RectAngleErrorList; // angle error
        for(int i = 0; i < RectList.size(); i++){
            float RectAngleError = 0;
            float RectAngle1 = RectList.at(i).angle;
            for(int j = 0; j < RectList.size(); j++ && j != i){
                float RectAngle2 = RectList.at(j).angle;
                float Error = fabs(RectAngle1 - RectAngle2);
                RectAngleError += Error;
            }
            RectAngleErrorList.push_back(RectAngleError);
        }

        std::vector<float> RectErrorList; // error
        for(int i = 0; i < RectList.size(); i++)
            RectErrorList.push_back(RectSizeErrorList.at(i) + RectAngleErrorList.at(i));
        for(int i = RectErrorList.size() - 2; i >= 0; i--) {
            for(int j = 0; j <= i; j++) {
                if(RectErrorList.at(j + 1) < RectErrorList.at(j)){
                    std::swap(RectErrorList.at(j + 1), RectErrorList.at(j));
                    std::swap(RectList.at(j + 1), RectList.at(j));
                    std::swap(PointList.at(j + 1), PointList.at(j));
                    std::swap(OutquadList.at(j + 1), OutquadList.at(j));
                }
            }
        }

        while(RectList.size() > 3)
            RectList.pop_back();
        while(RectList.size() > 3)
            PointList.pop_back();
        while(RectList.size() > 3)
            OutquadList.pop_back();
    } else if (RectList.size() <= 3) {
        printf("Profile Filter Successfully!\n");
    }
}

void QRCode::CornerSort(std::vector<cv::RotatedRect>& RectList, VecPoint2f& PointList) {
    std::vector<float> AngleList;

    for (int i = 0; i < PointList.size(); i++) {
        // compute every corner's angle
        float angle = 0;
        auto thisPoint = PointList.at(i);
        cv::Point2f otherPoint[2];
        if (i == 0) {
            otherPoint[0] = PointList.at(1);
            otherPoint[1] = PointList.at(2);
        } else if (i == 1) {
            otherPoint[0] = PointList.at(0);
            otherPoint[1] = PointList.at(2);
        } else if (i == 2) {
            otherPoint[0] = PointList.at(0);
            otherPoint[1] = PointList.at(1);
        }
        
        // compute the angle
        float a = sqrt(pow(thisPoint.x - otherPoint[1].x, 2) + 
                       pow(thisPoint.y - otherPoint[1].y, 2));     // a -- otherpoint[0]
        float b = sqrt(pow(otherPoint[0].x - otherPoint[1].x, 2) + 
                       pow(otherPoint[0].y - otherPoint[1].y, 2)); // b -- thispoint
        float c = sqrt(pow(thisPoint.x - otherPoint[0].x, 2) + 
                       pow(thisPoint.y - otherPoint[0].y, 2));     // c -- otherpoint[1]
        angle = acos((a * a + c * c - b * b) / 2 * a * c) * 180 / M_PI;
        AngleList.push_back(angle);
    }

    // find 0th corner
    for(int i = AngleList.size() - 2; i >= 0; i--) {
        for(int j = 0; j <= i; j++)
        {
            float error1 = fabs(AngleList.at(j) - 90);
            float error2 = fabs(AngleList.at(j + 1) - 90);
            if(error2 < error1){
                std::swap(AngleList.at(j + 1), AngleList.at(j));
                std::swap(PointList.at(j + 1), PointList.at(j));
                std::swap(RectList.at(j + 1), RectList.at(j));
            }
        }
    }
    
    // ensure 1st and 2nd corner
    // make two vectors: 0-->1 0-->2
    auto Angle = GetAngleOfTwoVector(PointList.at(1), PointList.at(2), PointList.at(0));
    if (Angle < 0)
        std::swap(PointList.at(1), PointList.at(2));

    // insert to map struct
    for (int i = 0; i < PointList.size(); i++) {
        corner_centers_.insert(CornerIndex(i, PointList.at(i)));
    }
}

void QRCode::Location(VecPoint2f& PointList, VecVecPoint2f& OutquadList) {

    // get qr_code center
    cv::Point2f center;
    center.x = 0.5 * (PointList.at(1).x + PointList.at(2).x);
    center.y = 0.5 * (PointList.at(1).y + PointList.at(2).y);

    // get corner points
    VecPoint2f CornerPointList;
    for (int i = 0; i < OutquadList.size(); i++) {
        VecPoint2f Points(OutquadList.at(i).size());
        Points = OutquadList.at(i);
        for (int j = 0; j < Points.size(); j++)
            CornerPointList.push_back(Points[j]);
    }

    // sort corner points
    auto Corner_0 = SortNearToFar(CornerPointList, PointList.at(0), 4); // 4 sub_corners near corner0
    auto Corner_1 = SortNearToFar(CornerPointList, PointList.at(1), 4); // near corner1
    auto Corner_2 = SortNearToFar(CornerPointList, PointList.at(2), 4); // near corner2

    // sort every block
    int i = 0;
    // corner 0
    auto corner_0_0 =  GetFarestPoint(Corner_0, center);
    auto angle = GetAngleOfTwoVector(Corner_0.at(1), center, GetFarestPoint(Corner_0, center));
    auto corner_0_1 = (angle < 0) ? Corner_0.at(2) : Corner_0.at(1);
    auto corner_0_2 = (angle < 0) ? Corner_0.at(1) : Corner_0.at(2);
    auto corner_0_3 = Corner_0.at(0);
    sub_corners_.insert(CornerIndex(i++, corner_0_0));
    sub_corners_.insert(CornerIndex(i++, corner_0_1));
    sub_corners_.insert(CornerIndex(i++, corner_0_2));
    sub_corners_.insert(CornerIndex(i++, corner_0_3));

    // corner 1
    auto corner_1_2 = GetNearestPoint(Corner_1, center);
    angle = GetAngleOfTwoVector(Corner_1.at(1), corner_1_2, center);
    auto corner_1_0 = (angle < 0) ? Corner_1.at(2) : Corner_1.at(1);
    auto corner_1_3 = (angle < 0) ? Corner_1.at(1) : Corner_1.at(2);
    auto corner_1_1 = Corner_1.at(Corner_1.size() - 1);
    sub_corners_.insert(CornerIndex(i++, corner_1_0));
    sub_corners_.insert(CornerIndex(i++, corner_1_1));
    sub_corners_.insert(CornerIndex(i++, corner_1_2));
    sub_corners_.insert(CornerIndex(i++, corner_1_3));

    // corner 2
    auto corner_2_1 = GetNearestPoint(Corner_2, center);
    angle = GetAngleOfTwoVector(center, corner_2_1, Corner_2.at(1));
    auto corner_2_0 = (angle < 0) ? Corner_2.at(2) : Corner_2.at(1);
    auto corner_2_3 = (angle < 0) ? Corner_2.at(1) : Corner_2.at(2);
    auto corner_2_2 = Corner_2.at(Corner_2.size() - 1);
    sub_corners_.insert(CornerIndex(i++, corner_2_0));
    sub_corners_.insert(CornerIndex(i++, corner_2_1));
    sub_corners_.insert(CornerIndex(i++, corner_2_2));
    sub_corners_.insert(CornerIndex(i++, corner_2_3));

    // corner 3
    auto corner_3_0 = GetCrossPoint(corner_1_0, corner_1_2, corner_2_0, corner_2_1);
    auto corner_3_1 = GetCrossPoint(corner_1_1, corner_1_3, corner_2_0, corner_2_1);
    auto corner_3_2 = GetCrossPoint(corner_1_0, corner_1_2, corner_2_2, corner_2_3);
    auto corner_3_3 = GetCrossPoint(corner_1_1, corner_1_3, corner_2_2, corner_2_3);
    sub_corners_.insert(CornerIndex(i++, corner_3_0));
    sub_corners_.insert(CornerIndex(i++, corner_3_1));
    sub_corners_.insert(CornerIndex(i++, corner_3_2));
    sub_corners_.insert(CornerIndex(i++, corner_3_3));

    // 4 corner centers
    i = 0;
    auto P0 = cv::Point2f((corner_0_0.x + corner_0_1.x + corner_0_2.x + corner_0_3.x) / 4,
                          (corner_0_0.y + corner_0_1.y + corner_0_2.y + corner_0_3.y) / 4);
    auto P1 = cv::Point2f((corner_1_0.x + corner_1_1.x + corner_1_2.x + corner_1_3.x) / 4,
                          (corner_1_0.y + corner_1_1.y + corner_1_2.y + corner_1_3.y) / 4);
    auto P2 = cv::Point2f((corner_2_0.x + corner_2_1.x + corner_2_2.x + corner_2_3.x) / 4,
                          (corner_2_0.y + corner_2_1.y + corner_2_2.y + corner_2_3.y) / 4);
    auto P3 = cv::Point2f((corner_3_0.x + corner_3_1.x + corner_3_2.x + corner_3_3.x) / 4,
                          (corner_3_0.y + corner_3_1.y + corner_3_2.y + corner_3_3.y) / 4);

    corner_centers_.insert(CornerIndex(i++, P0));
    corner_centers_.insert(CornerIndex(i++, P1));
    corner_centers_.insert(CornerIndex(i++, P2));
    corner_centers_.insert(CornerIndex(i++, P3));

    center_ = cv::Point2f((P0.x + P1.x + P2.x + P3.x) / 4,
                          (P0.y + P1.y + P2.y + P3.y) / 4);
}

VecPoint2f QRCode::Point2Point2f(const std::vector<cv::Point> VecPoint) {
    VecPoint2f vec_point2f(VecPoint.size());
    for (int i = 0; i < VecPoint.size(); i++) {
        vec_point2f.at(i) = cv::Point2f(VecPoint.at(i));
    }

    return vec_point2f;
}