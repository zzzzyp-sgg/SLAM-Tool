#include "Camera.h"

bool Camera::readParameters(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    bool tmp = fs.isOpened();
    if (!tmp)
    {
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType.compare("PINHOLE") != 0)
        {
            return false;
        }
    }

    mImageWidth = static_cast<int>(fs["image_width"]);
    mImageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["distortion_parameters"];
    mK1 = static_cast<double>(n["k1"]);
    mK2 = static_cast<double>(n["k2"]);
    mP1 = static_cast<double>(n["p1"]);
    mP2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    mFx = static_cast<double>(n["fx"]);
    mFy = static_cast<double>(n["fy"]);
    mCx = static_cast<double>(n["cx"]);
    mCy = static_cast<double>(n["cy"]);
    

    return true;
}