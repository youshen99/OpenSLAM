/*
 * System.h
 *
 *  Created on: 2016年12月15日
 *      Author: glodon
 */
#pragma once
#pragma once
#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

namespace YOUSHEN_SLAM{
class System {
public:
    enum eSensor{
        MONOCULAR=0,//单目
        STEREO=1,//双目
    };
public:
	System(const string &settingPath,const eSensor sensor);//初始化的过程仅仅初始化对应的设置文件的路径
	virtual ~System();
	//initSystem(const string &settingPath);

};
}
#endif /* SYSTEM_H_ */
