# QR码检测  
目前主要实现了两种情况  
  
## 1 利用opencv自带的cv::QRCodeDetector类  
基于OpenCV4实现，利用cv::QRCodeDetector类自带的detect函数来检测QR码，  
不过这种方式只能得到四个顶点的位置，而且看起来精度不是特别高  
对应函数 QRCode::DetectQuirc()  
  
## 2 利用OpenCV其他函数实现  
该方法针对模糊、二维码较远的情况下无法成功检测，对图像质量依赖较为严重  
对应函数 QRCode::DetectCode()