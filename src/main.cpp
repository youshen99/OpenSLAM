#include <iostream>
#include <opencv2/opencv.hpp>
#include "ORBextractor.h"
#include "CommonAccessMethod.h"
#include "Frame.h"
#include "Binocular.h"
using namespace std;
using namespace cv;
using namespace YOUSHEN_SLAM;
void testHanshu()
{
  Mat image = imread("/home/glodon/桌面/0003.png");
  //相机矩阵
  Mat K(Matx33d(2759.48, 0, 1520.69, 0, 2764.16, 1006.81, 0, 0, 1));
  Mat disCof = (Mat_<double>(1, 5) << 0, 0, 0, 0, 0); //畸变函数
  //第一张图片
  Frame *imageFrame = new Frame(image, K, disCof);
  //获取畸变矫正之后的点
  vector<KeyPoint> keyPointList1 = imageFrame->g_imageKeyPointUn;
  Mat desc1 = imageFrame->g_desc;
  //第二张图片
  Mat image2 = imread("");
  Frame *imageFrame2 = new Frame(image2, K, disCof);

  vector<KeyPoint> keyPointList2 = imageFrame2->g_imageKeyPointUn;
  Mat desc2 = imageFrame2->g_desc;
  //计算匹配度
  BFMatcher matcher(NORM_HAMMING);
  vector<DMatch> matches;
  matcher.match(desc1, desc2, matches);

  //初始化对应的函数
  CommonAccessMethod commonAN;

  //commonAN.findHomography();

  //将获取到的点进行画到图像中
  Mat outImage;
  drawKeypoints(image, keyPointList1, outImage);
  imshow("KeyPoint", outImage);
  cout << disCof << endl;
  waitKey(0);
  //YOUSHEN_SLAM::Frame myFrame(image,K,disCof);
  //Frame myFrame(image，K,disCof);
};

int binocularTest{
    // YOUSHEN_SLAM::binocular binoc();

};

int main()
{
  Binocular binoc;
  binoc.start();
  return 0;
}