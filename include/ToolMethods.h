/*
 * @Author: youshen 
 * @Date: 2017-01-05 18:45:57 
 * @Last Modified by:   youshen 
 * @Last Modified time: 2017-01-05 18:45:57 
 */
#ifndef TOOLMETHODS_H
#define TOOLMETHODS_H
#include<opencv2/opencv.hpp>
#include<string>
#include<vector>
namespace YOUSHEN_SLAM
{
   class ToolMethods{
      public:
         //该函数主要是将彩色图片转化为灰度图片，若是原图片是灰度图片，则进行忽略
        static void isImageToGrat(cv::Mat&image,cv::Mat&gray);
        //根据传来的配置文件路径，获取相机的参数，mk为相机的内参，mDisrCoef为相机的畸变参数
        void static getSettingFile(std::string settingPath,cv::Mat&mk,cv::Mat&mDistCoef);
       //反向和正向计算特征点的匹配 增加精度   参数一 匹配较长的个数   参数二是其次的   参数三 是返回匹配后的参数
        static  void computerMatch2(std::vector<cv::DMatch>&dMatch12,std::vector<cv::DMatch>&dMatch22,std::vector<cv::DMatch>&dMatchLaster2);
        //参数含义同上 前两个参数顺序可以无序
        static void  computerMatchPoint(std::vector<cv::DMatch>&dMatch1,std::vector<cv::DMatch>&dMatch2,std::vector<cv::DMatch>&dMatchLaster);
};
}

#endif /* TOOLMETHODS_H */