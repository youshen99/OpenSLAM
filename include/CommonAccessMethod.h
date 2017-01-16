/*
 * CommonAccessMethod.h
 *
 *  Created on: 2016年12月19日
 *      Author: glodon
 */

#ifndef COMMONACCESSMETHOD_H_
#define COMMONACCESSMETHOD_H_
#include<iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include<time.h>
#include<random>
#define	RAND_MAX	2147483647

using namespace cv;
using namespace std;
namespace YOUSHEN_SLAM {

class CommonAccessMethod {

public:
	CommonAccessMethod();
	virtual ~CommonAccessMethod();
    typedef pair<int,int> Match;
	void vectorDmatchToMatch(vector<DMatch>& vecDMatch,vector<Match>& vecMatch);
	//计算单应性矩阵
	Mat computerHomography(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    //计算基础矩阵
	Mat computeFundamental(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2);
	void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);
	//使用迭代的方法计算最好的H矩阵
// 参数：两幅图片匹配的特征点 匹配过程中符合匹配点的视为在其中的true，否则为false  最好的H矩阵的分数  H矩阵  mMaxIterations 最大迭代次数
   void findHomography(vector<KeyPoint>&mvKeys1,vector<KeyPoint> &mvKeys2,vector<DMatch> &mvMatches12,vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21,const int mMaxIterations,int mSigma);
    void findFundamental(vector<KeyPoint>&mvKeys1,vector<KeyPoint> &mvKeys2,vector<DMatch> &mvMatches12,vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21,const int mMaxIterations,int mSigma);

	void findHomography(vector<KeyPoint>&mvKeys1,vector<KeyPoint> &mvKeys2,vector<Match> &mvMatches12,vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21,const int mMaxIterations,int mSigma);
    void findFundamental(vector<KeyPoint>&mvKeys1,vector<KeyPoint> &mvKeys2,vector<Match> &mvMatches12,vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21,const int mMaxIterations,int mSigma);
	//获取一个集合  该集合是
	void getRandomSet(const int N,const int mMaxIterations,vector<vector<size_t> >&mvSets);
	int randomInt(int min, int max);
	//计算重投影误差
	//参数： 特征点1  特征点2  匹配参数  H矩阵  反向 H矩阵 内连匹配参数  阈值
	float CheckHomography(vector<KeyPoint>&mvKeys1,vector<KeyPoint>&mvKeys2,vector<Match> &mvMatches12,const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
    //通过F矩阵和H矩阵估计运动
//从H矩阵中回复R和T
//回复矩阵的方式有两种，分别是Faugeras SVD和Zhan SVD  
//参数： 内联点   H矩阵  相机内参矩阵   最小视差  最小三角重建精度   待求 旋转矩阵 位移矩阵  三角重建后的点 和 是否进行三维重建
	bool ReconstructH( vector<cv::KeyPoint> &mvKeys1, vector<cv::KeyPoint> &mvKeys2,vector<DMatch> &mvMatches12,float mSigma2,vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
	int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);
// 通过三角化方法，利用反投影矩阵将特征点恢复为3D点
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

};

} /* namespace YOUSHEN_SLAM */

#endif /* COMMONACCESSMETHOD_H_ */
