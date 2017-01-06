/*
 * @Author: youshen 
 * @Date: 2016-12-30 11:11:53 
 * @Last Modified by: youshen
 * @Last Modified time: 2017-01-05 19:07:06
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include "ORBextractor.h"
#include "CommonAccessMethod.h"
#include "Frame.h"
#include "Binocular.h"
#include "ToolMethods.h"
using namespace std;
using namespace cv;
using namespace YOUSHEN_SLAM;
void testHanshu()
{
  Mat image = imread("/home/glodon/imageShuang/left_image/000000.png");
  //相机矩阵  獲取相機的参数和畸变函数
  Mat K;
  Mat disCof;
  YOUSHEN_SLAM::ToolMethods::getSettingFile("/home/glodon/MySLAM/ORB_SLAM2/Examples/Monocular/MYdATA.yaml",K,disCof);
  //第一张图片
  Frame *imageFrame = new Frame(image, K, disCof);
  //获取畸变矫正之后的点
  vector<KeyPoint> keyPointList1 = imageFrame->g_imageKeyPointUn;
  Mat desc1 = imageFrame->g_desc;
  //第二张图片
  Mat image2 = imread("/home/glodon/imageShuang/left_image/000002.png");
  Frame *imageFrame2 = new Frame(image2, K, disCof);

  vector<KeyPoint> keyPointList2 = imageFrame2->g_imageKeyPointUn;
  Mat desc2 = imageFrame2->g_desc;
  //计算匹配度  需要进行优化处理 
  BFMatcher matcher(NORM_HAMMING);
  vector<DMatch> matches;
  matcher.match(desc1, desc2, matches);
  //DMatch和Match 转换
  //计算一个反响投影  控制特征点的匹配参数 增加精确度
   vector<DMatch> matches2;
   matcher.match(desc2, desc1, matches2);

vector<DMatch> dMatchAChoosel;
YOUSHEN_SLAM::ToolMethods::computerMatchPoint(matches2,matches,dMatchAChoosel);
  cout<<matches.size()<<"  "<<matches2.size()<<"    "<<dMatchAChoosel.size()<<endl;
  //初始化对应的函数
  CommonAccessMethod commonAN;
  vector<bool> vbMatchesInliersH;
  float scoreH;
  Mat H21;
  commonAN.findHomography(keyPointList1,keyPointList2,dMatchAChoosel,vbMatchesInliersH,scoreH,H21,200,1);
  vector<bool> vbMatchesInliersF;
  float scoreF;
  Mat F21;
  commonAN.findFundamental(keyPointList1,keyPointList2,dMatchAChoosel,vbMatchesInliersF,scoreF,F21,200,1);
  cout<<"H:  "<<scoreH<<"  F:  "<<scoreF<<endl;
  float RH = scoreH/(scoreH+scoreF);
  cout<<RH<<endl;
  //将获取到的点进行画到图像中
  Mat outImage;
  drawKeypoints(image, keyPointList1, outImage);
  imshow("KeyPoint", outImage);
  cout << disCof << endl;

    Mat imageMatches;
    drawMatches(image,keyPointList1,image2,keyPointList2,dMatchAChoosel,
        imageMatches,Scalar(255,0,0));
 imshow("KeyPoint2", imageMatches);
  waitKey(0);
  //YOUSHEN_SLAM::Frame myFrame(image,K,disCof);
  //Frame myFrame(image，K,disCof);
};

int binocularTest{
    // YOUSHEN_SLAM::binocular binoc();

};
//入口函数
int main()
{

testHanshu();
//开启双目  获取数据
 // Binocular binoc;
//   binoc.start();
  return 0;
}