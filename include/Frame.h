/*
 * Frame.h
 *帧函数  主要含有对应的因子
 *  Created on: 2016年12月16日
 *      Author: glodon
 */

#ifndef FRAME_H_
#define FRAME_H_
#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include"ORBextractor.h"
using namespace std;
using namespace cv;

namespace YOUSHEN_SLAM {
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
// ORBextractor(int features_num = 500, float scale_factor = 1.2f, int levels_num = 8,
// 		int default_fast_threshold = 20, int min_fast_threshold = 7);
#define  features_num 500
#define  scale_factor 1.2f
#define  levels_num 8
#define  default_fast_threshold 20
#define  min_fast_threshold 7


class Frame {
public:
	//参数 ： 图片  相机内参矩阵3x3 图像矫正系数1x4
	Frame(Mat&image, Mat&K, Mat& distCoef);
	virtual ~Frame();
	void setIsFristFrame(bool);
	static float g_fx;
	static float g_fy;
	static float g_cx;
	static float g_cy;
	static float g_invfx;
	static float g_invfy;
	//主要是用来将图像分块处理
	// 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
	static float g_fGridElementWidthInv;
	static float g_fGridElementHeightInv;
	vector<KeyPoint> g_imageKeyPoint;
	//被矫正之后的特征点
	vector<KeyPoint> g_imageKeyPointUn;
	Mat g_desc;
private:
	//对应的图像信息
	Mat g_image;
	//相机的内参矩阵
	Mat g_mk;
	//图像矫正系数
	Mat g_disCoef;

	//矫正后图片的四个点
	float mnMinX = 0.0f;
	float mnMaxX = 0.0f;
	float mnMinY = 0.0f;
	float mnMaxY = 0.0f;
	//保存原始的相机矩阵数据
	
	// 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀
	// FRAME_GRID_ROWS 48
	// FRAME_GRID_COLS 64
	 vector<std::size_t> g_mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

	//特征点个数获取
	int g_keyPointCount;
	//判断是否是第一帧
	bool g_isInitFristFrame = true;
	//矫正函数
	void UndistortKeyPoints();
	//特征提取函数
	void extraction_feature(Mat &image, vector<KeyPoint> &keyPoint,
			Mat &description);
	//抽取图像的特征信息
	void extraction_DMatch(Mat &desc1, Mat &desc2, vector<DMatch>&match);
	//获取特征点个数
	int getKeyPointCount();
	//进行图像矫正
	void ComputeImageBounds(Mat &imGray);
	void AssignFeaturesToGrid();
	 bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
};

} /* namespace YOUSHEN_SLAM */

#endif /* FRAME_H_ */
