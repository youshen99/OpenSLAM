/*
 * System.cpp
 *
 *  Created on: 2016年12月15日
 *      Author: glodon
 */

#include "System.h"
namespace YOUSHEN_SLAM {

System::System(const string& settingPath, const eSensor sensor) {
	//通过文件读取对应的信息
	cv::FileStorage fsSetting(settingPath.c_str(), cv::FileStorage::READ); //读文件
	if (!fsSetting.isOpened()) {
		exit(-1);
	}

}

System::~System() {
	// TODO Auto-generated destructor stub
}

}
