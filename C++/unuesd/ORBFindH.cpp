/*******************************************************
 * Copyright (C) 2023, WuHan University
 * 
 * This file is just for development, and cannot be used yet.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: zyp
 *******************************************************/

#include "ORBFindH.h"
#include <cmath>

void FindH(const VecPoint2f &kp1, const VecPoint2f &kp2, float &score, cv::Mat &H21, VecBool &inlierH) {
    VecPoint2f kp1_n, kp2_n;
    cv::Mat T1, T2;
    Normalize(kp1, kp1_n, T1);
    Normalize(kp2, kp2_n, T2);

    cv::Mat T2inv = T2.inv();

    score = 0.0;
    int N = kp1.size();
    inlierH = VecBool(N, false);

    VecPoint2f pn1i(8), pn2i(8);
    cv::Mat H21i, H12i;

    VecBool current_inliers(N, false);
    float current_score;

    for (int it = 0; it < 10; it++) {
        for (size_t j = 0; j < 8; j++) {
            pn1i[j] = kp1_n[10 * it + j];
            pn2i[j] = kp2_n[10 * it + j];
        }

        auto Hn = ComputeH(pn1i, pn2i);
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();

        current_score = CheckHomography(H21i, H12i, kp1, kp2, 1.0, inlierH);
        
        if (current_score > score) {
            H21 = H21i.clone();
            score = current_score;
        }
    }
}

void Normalize(const VecPoint2f &kp, VecPoint2f &kpn, cv::Mat &T) {
    float meanX = 0;
    float meanY = 0;

    const int N = kp.size();

    kpn.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += kp[i].x;
        meanY += kp[i].y;
    }

    meanX = meanX / N;
    meanY = meanY / N;


    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        kpn[i].x = kp[i].x - meanX;
        kpn[i].y = kp[i].y - meanY;

        meanDevX += fabs(kpn[i].x);
        meanDevY += fabs(kpn[i].y);
    }

    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;
    float sX = 1.0 / meanDevX;
    float sY = 1.0 / meanDevY;

    for(int i = 0; i < N; i++)
    {
        kpn[i].x = kpn[i].x * sX;
        kpn[i].y = kpn[i].y * sY;
    }

    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

cv::Mat ComputeH(const VecPoint2f &vP1, const VecPoint2f &vP2) {
    const int N = vP1.size();

    cv::Mat A(2*N, 9, CV_32F);

    for(int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);


    return vt.row(8).reshape(0, 3);
}

float CheckHomography(const cv::Mat &H21, const cv::Mat &H12,
                                 const VecPoint2f &kp1, const VecPoint2f &kp2, float sigma, VecBool current_in) {
    const int N = kp1.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    current_in.resize(N);
    float score = 0;

    const float th = 5.991;

    const float invSigmaSquare = 1.0/(sigma * sigma);


    for(int i = 0; i < N; i++)
    {
        bool bIn = true;

        const auto &p1 = kp1[i];
        const auto &p2 = kp2[i];
        const float u1 = p1.x;
        const float v1 = p1.y;
        const float u2 = p2.x;
        const float v2 = p2.y;


        const float w2in1inv = 1.0/(h31inv * u2 + h32inv * v2 + h33inv);
        const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
        const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;
   
        const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);
        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;    
        else
            score += th - chiSquare1;


        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);
        const float chiSquare2 = squareDist2*invSigmaSquare;
 
        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;   

        if(bIn)
            current_in[i]=true;
        else
            current_in[i]=false;
    }
    return score;
}

bool ReconstructH(VecBool &inlierH, cv::Mat &H21, cv::Mat &K,  const VecPoint2f &kp1, const VecPoint2f &kp2,
                             cv::Mat &R21, cv::Mat &t21, VecPoint3d &points, VecBool &vbTriangulated, float minParallax, int minTriangulated) {
    int N=0;
    for(size_t i = 0, iend = inlierH.size() ; i < iend; i++)
        if(inlierH[i])
            N++;

    cv::Mat invK = K.inv();
    cv::Mat A = invK * H21 * K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A, w, U, Vt, cv::SVD::FULL_UV);

    V = Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);
    
    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1 / d2 < 1.00001 || d2 / d3 < 1.00001) {
        return false;
    }

    std::vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);
    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];        
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;

        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }
    
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);
    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    VecPoint3d bestP3D;
    VecBool bestTriangulated;

    // Instead of applying the visibility constraints proposed in the WFaugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        VecPoint3d vP3Di;
        VecBool vbTriangulatedi;
    
        int nGood = CheckRT(vR[i],vt[i],kp1,kp2,inlierH,K,
                            vP3Di,4.0,vbTriangulatedi, parallaxi);
        
        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }

    if(secondBestGood<0.75*bestGood &&      
       bestParallax>=minParallax &&
       bestGood>minTriangulated && 
       bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        points = bestP3D;
        vbTriangulated = bestTriangulated;
        return true;
    }
    return false;
}

int CheckRT(const cv::Mat &R, const cv::Mat &t, const VecPoint2f &vKeys1, const VecPoint2f &vKeys2, 
                       VecBool &vbMatchesInliers, const cv::Mat &K, VecPoint3d &vP3D, float th2, VecBool &vbGood, float &parallax)
{
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = VecBool(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    std::vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));
    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;
    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vKeys1.size();i<iend;i++)
    {

        if(!vbMatchesInliers[i])
            continue;

        const auto &kp1 = vKeys1[i];
        const auto &kp2 = vKeys2[i];
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);	

        if(!std::isfinite(p3dC1.at<float>(0)) || !std::isfinite(p3dC1.at<float>(1)) || !std::isfinite(p3dC1.at<float>(2)))
        {
            vbGood[i]=false;
            continue;
        }

        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);


        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
        // if(p3dC1.at<float>(2)<=0 || cosParallax>0.99998)
            continue;


        cv::Mat p3dC2 = R*p3dC1+t;	
        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.x)*(im1x-kp1.x)+(im1y-kp1.y)*(im1y-kp1.y);

        if(squareError1>th2)
            continue;

        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.x)*(im2x-kp2.x)+(im2y-kp2.y)*(im2y-kp2.y);

        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[i] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[i]=true;
            nGood++;
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = std::min(50,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

void Triangulate(const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}