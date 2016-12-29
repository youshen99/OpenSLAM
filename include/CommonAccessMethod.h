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
// 参数：两幅图片匹配的特征点   最好的H矩阵的分数  H矩阵  mMaxIterations 最大迭代次数
	void findHomography(vector<KeyPoint>&mvKeys1,vector<KeyPoint> &mvKeys2,vector<Match> &mvMatches12,vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21,const int mMaxIterations,int mSigma);
    void findFundamental(vector<KeyPoint>&mvKeys1,vector<KeyPoint> &mvKeys2,vector<Match> &mvMatches12,vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21,const int mMaxIterations,int mSigma);
	//获取一个集合  该集合是
	void getRandomSet(const int N,const int mMaxIterations,vector<vector<size_t> >&mvSets);
	int randomInt(int min, int max);
	//计算重投影误差
	//参数： 特征点1  特征点2  匹配参数  H矩阵  反向 H矩阵 内连匹配参数  阈值
	float CheckHomography(vector<KeyPoint>&mvKeys1,vector<KeyPoint>&mvKeys2,vector<Match> &mvMatches12,const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

};

} /* namespace YOUSHEN_SLAM */

#endif /* COMMONACCESSMETHOD_H_ */
