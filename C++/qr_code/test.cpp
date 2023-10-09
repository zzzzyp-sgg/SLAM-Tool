/*******************************************************
 * Copyright (C) 2023, WuHan University
 * 
 * This file is for test QR code detect function.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: zyp
 *******************************************************/

#include "qr_code.h"
#include "qr_math.h"
#include "../initial/frame.h"
#include "../initial/Camera.h"
#include "../initial/estimator.h"
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>


void display(cv::Mat &im, VecPoint2f& corners)
{
	int n = corners.size();
	for (int i = 0; i < n; i++)
	{
		cv::circle(im, corners.at(i), 10, (255, 0, 0));
	}
	// imshow("Result", im);
}

int main(int argc, char **argv) {
    if (argc != 5) {
        std::cout << "Input: image_path img2_path config num!" << std::endl;
        exit(0);
    }

    std::string ImgPath = argv[1], ImgPath2 = argv[2], config = argv[3];
    int n = std::stoi(argv[4]);
    cv::Mat src_img = cv::imread(ImgPath, cv::IMREAD_GRAYSCALE);
    cv::Mat src_img2 = cv::imread(ImgPath2, cv::IMREAD_GRAYSCALE);

#if 0

    int row_0 = src_img.rows / n;
    int col_0 = src_img.cols / n;
    auto tmp_img = (n == 1) ? src_img : src_img(cv::Range(row_0, (n - 1) * row_0), cv::Range(col_0, (n - 1) * col_0));
    // cv::imshow("detect image", tmp_img);
    // cv::waitKey(0);
    auto t1 = std::chrono::steady_clock::now();
    cv::QRCodeDetector qrDecoder = cv::QRCodeDetector();
 
	//二维码边框坐标，提取出来的二维码
	// cv::Mat bbox, rectifiedImage;
    VecPoint2f corners;
 
	//检测二维码
	// std::string data = qrDecoder.detectAndDecode(tmp_img, bbox, rectifiedImage);
    auto success = qrDecoder.detect(tmp_img, corners);
    auto t2 = std::chrono::steady_clock::now();
    auto tt = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    printf("detect costs %ld ms\n", tt.count());

    if (success) {
        display(tmp_img, corners);
		// rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
		//展示二维码
		cv::imshow("Detect QRCode", tmp_img);
 
		cv::waitKey(0);
    } else {
        printf("Detect failed!\n");
    }
#endif

#if 0

    std::shared_ptr<QRCode> qr(new QRCode(src_img, n));
    // detect qr code
    qr->DetectQuirc();

    // detect keypoints
    qr->DetectFeature();

#endif

#if 1
    std::shared_ptr<Frame> f(new Frame(src_img, n));
    // f->Display();
    std::shared_ptr<Frame> f2(new Frame(src_img2, n));
    // f2->Display();
    std::shared_ptr<Camera> cam(new Camera());
    cam->readParameters(config);
    std::shared_ptr<Estimator> est(new Estimator(cam));
    est->AddFrame(f);
    est->AddFrame(f2);
    est->EstProcssing();
#endif
    return 0;
}