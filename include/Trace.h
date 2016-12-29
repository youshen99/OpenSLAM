/*
 * Trace.h
 *
 *  Created on: 2016年12月16日
 *      Author: glodon
 */

#ifndef TRACE_H_
#define TRACE_H_
#include<opencv2/opencv.hpp>
#include<iostream>
#include"ORBextractor.h"
using namespace std;
using namespace cv;
namespace YOUSHEN_SLAM {

class Trace {
public:
	Trace(const string &strSettingPath);
	virtual ~Trace();
//设置图片传入入口
	void push_Image(Mat&);
public:
	//相机矩阵
	Mat g_mK;
	//相机的畸变参数
	Mat g_mDistCoef;
};

} /* namespace YOUSHEN_SLAM */

#endif /* TRACE_H_ */
