/*
 * Trace.cpp
 *
 *  Created on: 2016年12月16日
 *      Author: glodon
 */

#include "Trace.h"

namespace YOUSHEN_SLAM {
//参数:设置函数的路径
Trace::Trace(const string &strSettingPath) {
//初始化对应的函数
	  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
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
	    //复制到全局变量中
	    K.copyTo(g_mK);

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
	    DistCoef.copyTo(g_mDistCoef);
}

Trace::~Trace() {
	// TODO Auto-generated destructor stub
}
void Trace::push_Image(Mat &){
//若是第一个传入的图片则等待下一个图片的进入



}


} /* namespace YOUSHEN_SLAM */
