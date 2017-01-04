/*
 * @Author: youshen 
 * @Date: 2016-12-30 11:11:53 
 * @Last Modified by:   youshen 
 * @Last Modified time: 2016-12-30 11:11:53 
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include "ORBextractor.h"
#include "CommonAccessMethod.h"
#include "Frame.h"
#include "Binocular.h"
using namespace std;
using namespace cv;
using namespace YOUSHEN_SLAM;
//转换灰度图像 

void isImageToGrat(Mat&image,Mat&gray){
	if(image.type()==CV_8UC1){
       gray =image;
	}else{
       cvtColor(image,gray,CV_BGR2GRAY);
	}
}
/**
通过文件获取相机的内参和畸变函数
*/
void getSettingFile(string settingPath,Mat&mk,Mat&mDistCoef){
 cv::FileStorage fSettings(settingPath.c_str(), cv::FileStorage::READ);
//路径无效的情况
if(!fSettings.isOpened()){
      cerr << "Failed to open settings file at: " << settingPath << endl;
      exit(-1);
}
//读取对应的参数文件
 float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    //     |fx  0   cx|
    // K = |0   fy  cy|
    //     |0   0   1 |
    //构建相机内参矩阵
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mk); 
     // 图像矫正系数
    //获取图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);
}

//前者的長度比後者要大
       void computerMatch2(vector<DMatch>&dMatch12,vector<DMatch>&dMatch22,vector<DMatch>&dMatchLaster2){
               for (size_t i = 0; i < dMatch12.size(); i++)
               {
                    for (size_t j = 0; j < dMatch22.size(); j++)
               {//判断对应的前后只是否是一样的
                   if(dMatch12[i].queryIdx==dMatch22[j].trainIdx){
                     if(dMatch22[j].queryIdx==dMatch12[i].trainIdx){
                         dMatchLaster2.push_back(dMatch12[i]);       
                        }
                      }
    
                 }
               }

        }

void computerMatchPoint(vector<DMatch>&dMatch1,vector<DMatch>&dMatch2,vector<DMatch>&dMatchLaster){
     //计算输入的特征点的长度
     if(dMatch1.size()<dMatch2.size()){
       
         computerMatch2(dMatch2,dMatch1,dMatchLaster);
     }else{
         computerMatch2(dMatch1,dMatch2,dMatchLaster);
     }
}

void testHanshu()
{
  Mat image = imread("/home/glodon/imageShuang/left_image/000000.png");
  //相机矩阵  獲取相機的参数和畸变函数
  Mat K;
  Mat disCof;

  getSettingFile("/home/glodon/MySLAM/ORB_SLAM2/Examples/Monocular/MYdATA.yaml",K,disCof);


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
computerMatchPoint(matches2,matches,dMatchAChoosel);
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
//入口函数
int main()
{

testHanshu();

//     Mat image = imread("/home/glodon/imageShuang/left_image/000000.png");
//     Mat gray;
//     isImageToGrat(image,gray);
//   // Binocular binoc;
//   // binoc.start();
// //使用特征点提取算法进行orb特征提取

// ORBextractor orb(500,1.2,8,20,7);
// Mat mask;
// vector<KeyPoint> keyPoint;
// Mat desc;
// orb(gray,mask,keyPoint,desc);
// Mat outputMat;
// drawKeypoints(image,keyPoint,outputMat,Scalar(255,0,0),DrawMatchesFlags::DEFAULT);
// imshow("dian",outputMat);

// cout<<keyPoint.size()<<endl;
// waitKey(0);
  return 0;
}