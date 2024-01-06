#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>

class Camera {
public:
    Camera() : mImageWidth(0), mImageHeight(0), mFx(0.0), mFy(0.0), mCx(0.0), mCy(0.0) {

    }

    bool readParameters(const std::string& config);

public:
    int mImageWidth, mImageHeight;
    double mFx, mFy, mCx, mCy;
    double mK1, mK2, mP1, mP2;
};

#endif