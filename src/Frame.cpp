/*/*
 * Frame.cpp
 *
 *  Created on: 2016年12月16日
 *      Author: glodon
 */

#include "Frame.h"

namespace YOUSHEN_SLAM {

     float Frame::g_fx=0;
	 float Frame::g_fy=0;
	 float Frame::g_cx=0;
	 float Frame::g_cy=0;
	 float Frame::g_invfx=0;
	 float Frame::g_invfy=0;
	//主要是用来将图像分块处理
	// 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
	 float  Frame::g_fGridElementWidthInv=0;
	 float  Frame::g_fGridElementHeightInv=0;

Frame::Frame(Mat&image, Mat&K, Mat& distCoef) {
	this->g_mk = K.clone();
	this->g_image = image.clone();
	g_disCoef = distCoef.clone();
    Mat imGray2;
	if(this->g_image.type()==CV_8UC1){
       imGray2 = this->g_image;
	}else{
       cvtColor(this->g_image,imGray2,CV_BGR2GRAY);
	}


	//对该帧提取特征点
	extraction_feature(image, this->g_imageKeyPoint, this->g_desc);
	//获取特征点的位置
	this->g_keyPointCount = this->g_imageKeyPoint.size();
	//对特征点进行矫正处理
	UndistortKeyPoints();
    //若是第一帧  进行初始画工作
	//判断是否是第一帧  进行图像矫正
	if (g_isInitFristFrame) {
		//计算图像边界
		ComputeImageBounds (imGray2);
		//
	
		Frame::g_fGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS)
				/ static_cast<float>(mnMaxX - mnMinX);
		Frame::g_fGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS)
				/ static_cast<float>(mnMaxY - mnMinY);

		Frame::g_fx = K.at<float>(0, 0);
		Frame::g_fy = K.at<float>(1, 1);
		Frame::g_cx = K.at<float>(0, 2);
		Frame::g_cy = K.at<float>(1, 2);
		Frame::g_invfx = 1.0f / Frame::g_fx;
		Frame::g_invfy = 1.0f / Frame::g_fy;

		g_isInitFristFrame = false;
	}

}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
}
//将原有的特征点分配到各格子中
void Frame::AssignFeaturesToGrid(){
	//计算每个格子应该保留的特征点数
   int nReserve = 0.5f*this->g_keyPointCount/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
   //初始化 每个格子的平均的个数
   for(unsigned int i=0;i<FRAME_GRID_COLS;i++){
      for(unsigned int j=0;j<FRAME_GRID_ROWS;j++){
    	  g_mGrid[i][j].reserve(nReserve);
      }
   }
   //记录对应的特征点
   for(int i=0;i<this->g_keyPointCount;i++){
     KeyPoint local_kp = g_imageKeyPointUn[i];
     int local_nGridPosX, local_nGridPosY;
//判断在什么位置
     if(PosInGrid(local_kp,local_nGridPosX,local_nGridPosY)){
    	 g_mGrid[local_nGridPosX][local_nGridPosX].push_back(i);
     }
   }
}
bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY){
      posX=round((kp.pt.x-mnMinX)*g_fGridElementWidthInv);
      posY = round((kp.pt.y-mnMinY)*g_fGridElementHeightInv);

      if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS){
    	  return false;
      }
      return true;
}
//设置是否是第一帧
void Frame::setIsFristFrame(bool isFtistFrame) {
	this->g_isInitFristFrame = isFtistFrame;
}

void Frame::ComputeImageBounds(Mat &imGray) {
//判断是否是灰度图像
	if (imGray.type() != CV_8UC1) {
		cvtColor(imGray, imGray, CV_BGR2GRAY);
	}
	//如果矫正的参数是0则表明该图像是无畸变的参数
	if (this->g_disCoef.at<float>(0) != 0.0) {
		Mat local_mat(4, 2, CV_32F);
		local_mat.at<float>(0, 0) = 0.0;         //左上
		local_mat.at<float>(0, 1) = 0.0;
		local_mat.at<float>(1, 0) = imGray.cols; //右上
		local_mat.at<float>(1, 1) = 0.0;
		local_mat.at<float>(2, 0) = 0.0;         //左下
		local_mat.at<float>(2, 1) = imGray.rows;
		local_mat.at<float>(3, 0) = imGray.cols; //右下
		local_mat.at<float>(3, 1) = imGray.rows;
		local_mat = local_mat.reshape(2);
		undistortPoints(local_mat, local_mat, this->g_mk, g_disCoef, Mat(),
				g_mk);
		local_mat = local_mat.reshape(1);

		mnMinX = min(local_mat.at<float>(0, 0), local_mat.at<float>(2, 0)); //左上和左下横坐标最小的
		mnMaxX = max(local_mat.at<float>(1, 0), local_mat.at<float>(3, 0)); //右上和右下横坐标最大的
		mnMinY = min(local_mat.at<float>(0, 1), local_mat.at<float>(1, 1)); //左上和右上纵坐标最小的
		mnMaxY = max(local_mat.at<float>(2, 1), local_mat.at<float>(3, 1)); //左下和右下纵坐标最小的
	} else {
		mnMinX = 0.0f;
		mnMaxX = imGray.cols;
		mnMinY = 0.0f;
		mnMaxY = imGray.rows;
	}

}

void Frame::UndistortKeyPoints() {
	//将特征点保存在对应的矩阵中
	cv::Mat local_mat(this->getKeyPointCount(), 2, CV_32F);
	for (int i = 0; i < this->g_imageKeyPoint.size(); i++) {
		local_mat.at<float>(i, 0) = this->g_imageKeyPoint[i].pt.x;
		local_mat.at<float>(i, 1) = this->g_imageKeyPoint[i].pt.y;
	}
	local_mat = local_mat.reshape(2);
	cv::undistortPoints(local_mat, local_mat, g_mk, this->g_disCoef, cv::Mat(),
			g_mk);
	local_mat = local_mat.reshape(1);

	//存储对应的矫正后的特征点

	this->g_imageKeyPointUn.resize(this->getKeyPointCount());
	for (int i = 0; i < this->getKeyPointCount(); i++) {
		KeyPoint kp = this->g_imageKeyPoint[i];
		kp.pt.x = local_mat.at<float>(i, 0);
		kp.pt.y = local_mat.at<float>(i, 1);
		this->g_imageKeyPointUn[i] = kp;
	}

}
//特征提取函数
void Frame::extraction_feature(Mat &image, vector<KeyPoint> &keyPoint,
		Mat &description) {
	ORBextractor orb(2 * features_num,scale_factor, levels_num,
			default_fast_threshold, min_fast_threshold);
	Mat gray;
	if (image.type() != CV_8UC1) {
		cvtColor(image, gray, CV_BGR2GRAY);
	} else {
		image.copyTo(gray);
	}
     Mat mask;
	orb(gray,mask,keyPoint,description);
}
//抽取图像的特征信息
void Frame::extraction_DMatch(Mat &desc1, Mat &desc2, vector<DMatch>&match) {
              
}

int Frame::getKeyPointCount() {
	return this->g_keyPointCount;
}

} /* namespace YOUSHEN_SLAM */
