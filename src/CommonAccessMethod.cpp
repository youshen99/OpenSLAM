/*
 * CommonAccessMethod.cpp
 *
 *  Created on: 2016年12月19日
 *      Author: glodon
 */

#include "CommonAccessMethod.h"

namespace YOUSHEN_SLAM
{

CommonAccessMethod::CommonAccessMethod()
{
    // TODO Auto-generated constructor stub
}

CommonAccessMethod::~CommonAccessMethod()
{
    // TODO Auto-generated destructor stub
}
Mat CommonAccessMethod::computeFundamental(const vector<cv::Point2f> &vP1,
					   const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size(); //推到中 是一维的矩阵  所以只要开辟对应的控件即可

    cv::Mat A(N, 9, CV_32F); // N*9

    for (int i = 0; i < N; i++)
    {
	const float u1 = vP1[i].x;
	const float v1 = vP1[i].y;
	const float u2 = vP2[i].x;
	const float v2 = vP2[i].y;

	A.at<float>(i, 0) = u2 * u1;
	A.at<float>(i, 1) = u2 * v1;
	A.at<float>(i, 2) = u2;
	A.at<float>(i, 3) = v2 * u1;
	A.at<float>(i, 4) = v2 * v1;
	A.at<float>(i, 5) = v2;
	A.at<float>(i, 6) = u1;
	A.at<float>(i, 7) = v1;
	A.at<float>(i, 8) = 1;
    }

    cv::Mat u, w, vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3); // v的最后一列

    cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV); //然后做在做一次

    w.at<float>(2) = 0; // 秩2约束，将第3个奇异值设为0

    return u * cv::Mat::diag(w) * vt;
}
Mat CommonAccessMethod::computerHomography(const vector<cv::Point2f> &vP1,
					   const vector<cv::Point2f> &vP2)
{

    const int N = vP1.size(); //点的个数

    cv::Mat A(2 * N, 9, CV_32F); // 2N*9   //初始化矩阵，通过公式可以发现 矩阵A两行，所以8个  则需要2倍空间

    for (int i = 0; i < N; i++) //每一次循环完成一个对应的一个A矩阵的赋值
    {
	const float u1 = vP1[i].x;
	const float v1 = vP1[i].y;
	const float u2 = vP2[i].x;
	const float v2 = vP2[i].y;

	A.at<float>(2 * i, 0) = 0.0;
	A.at<float>(2 * i, 1) = 0.0;
	A.at<float>(2 * i, 2) = 0.0;
	A.at<float>(2 * i, 3) = -u1;
	A.at<float>(2 * i, 4) = -v1;
	A.at<float>(2 * i, 5) = -1;
	A.at<float>(2 * i, 6) = v2 * u1;
	A.at<float>(2 * i, 7) = v2 * v1;
	A.at<float>(2 * i, 8) = v2;

	A.at<float>(2 * i + 1, 0) = u1;
	A.at<float>(2 * i + 1, 1) = v1;
	A.at<float>(2 * i + 1, 2) = 1;
	A.at<float>(2 * i + 1, 3) = 0.0;
	A.at<float>(2 * i + 1, 4) = 0.0;
	A.at<float>(2 * i + 1, 5) = 0.0;
	A.at<float>(2 * i + 1, 6) = -u2 * u1;
	A.at<float>(2 * i + 1, 7) = -u2 * v1;
	A.at<float>(2 * i + 1, 8) = -u2;
    }

    cv::Mat u, w, vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV); //进行奇异值分解

    return vt.row(8).reshape(0, 3); // v的最后一列  选取最小的作为目前的真值
}
//进行归一话处理
/**
 * ＠brief 归一化特征点到同一尺度（作为normalize DLT的输入）
 *
 * [x' y' 1]' = T * [x y 1]' \n
 * 归一化后x', y'的均值为0，sum(abs(x_i'-0))=1，sum(abs((y_i'-0))=1
 *
 * @param vKeys             特征点在图像上的坐标
 * @param vNormalizedPoints 特征点归一化后的坐标
 * @param T                 将特征点归一化的矩阵
 */
void CommonAccessMethod::Normalize(const vector<cv::KeyPoint> &vKeys,
				   vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);
    for (int i = 0; i < N; i++)
    {
	meanX += vKeys[i].pt.x;
	meanY += vKeys[i].pt.y;
    }
    meanX = meanX / N;
    meanY = meanY / N;
    //计算尺度因子
    float meanDevX = 0;
    float meanDevY = 0;
    // 将所有vKeys点减去中心坐标，使x坐标和y坐标均值分别为0
    for (int i = 0; i < N; i++)
    {
	vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
	vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

	meanDevX += fabs(vNormalizedPoints[i].x);
	meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;

    float sX = 1.0 / meanDevX;
    float sY = 1.0 / meanDevY;

    // 将x坐标和y坐标分别进行尺度缩放，使得x坐标和y坐标的一阶绝对矩分别为1
    for (int i = 0; i < N; i++)
    {
	vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
	vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }
    //特征点归一话后的矩阵
    // |sX  0  -meanx*sX|
    // |0   sY -meany*sY|
    // |0   0      1    |
    T = cv::Mat::eye(3, 3, CV_32F);
    T.at<float>(0, 0) = sX;
    T.at<float>(1, 1) = sY;
    T.at<float>(0, 2) = -meanX * sX;
    T.at<float>(1, 2) = -meanY * sY;
}
int CommonAccessMethod::randomInt(int min, int max)
{
    int d = max - min + 1;
    return int(((double)rand() / ((double)RAND_MAX + 1.0)) * d) + min;
}
void CommonAccessMethod::getRandomSet(const int N, const int mMaxIterations,
				      vector<vector<size_t>> &mvSets)
{
    // 新建一个容器vAllIndices，生成0到N-1的数作为特征点的索引
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for (int i = 0; i < N; i++)
    {
	vAllIndices.push_back(i);
    }
    //在所有匹配点中随机选取八对匹配点，一共最大迭代次数次
    mvSets = vector<vector<size_t>>(mMaxIterations, vector<size_t>(8, 0));
    //DUtils::Random::SeedRandOnce(0);
    for (int it = 0; it < mMaxIterations; it++)
    {
	vAvailableIndices = vAllIndices;
	for (size_t j = 0; j < 8; j++)
	{
	    // 产生0到N-1的随机数
	    int randi = randomInt(0, vAvailableIndices.size() - 1);
	    // idx表示哪一个索引对应的特征点被选中
	    int idx = vAvailableIndices[randi];

	    mvSets[it][j] = idx;

	    // randi对应的索引已经被选过了，从容器中删除
	    // randi对应的索引用最后一个元素替换，并删掉最后一个元素
	    vAvailableIndices[randi] = vAvailableIndices.back();
	    vAvailableIndices.pop_back();
	}
    }
}
// void CommonAccessMethod::findFundamental(vector<KeyPoint> &mvKeys1,
// 					 vector<KeyPoint> &mvKeys2, vector<Match> &mvMatches12,
// 					 vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21,
// 					 const int mMaxIterations, int mSigma)
// {




// }
void CommonAccessMethod::vectorDmatchToMatch(vector<DMatch> &vecDMatch1, vector<Match> &vecMatch)
{
    for (size_t i = 0; i < vecDMatch1.size(); i++)
    {
		Match match;
	match.first = vecDMatch1[i].queryIdx;
	match.second = vecDMatch1[i].trainIdx;
    //cout<<vecDMatch1[i].queryIdx<<"   "<<vecDMatch1[i].trainIdx<<endl;
	vecMatch.push_back(match);
    }
}
//其中 vbMatchesInliers socre H21 是带球的点  最大迭代次数  默认200 mSigma 为 1.0
void CommonAccessMethod::findHomography(vector<KeyPoint> &mvKeys1, vector<KeyPoint> &mvKeys2, vector<DMatch> &mvMatches12, vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21, const int mMaxIterations, int mSigma)
{
    vector<Match> match12;
    vectorDmatchToMatch(mvMatches12, match12);
    findHomography(mvKeys1, mvKeys2, match12, vbMatchesInliers, score, H21, mMaxIterations, mSigma);
}
void CommonAccessMethod::findFundamental(vector<KeyPoint> &mvKeys1, vector<KeyPoint> &mvKeys2, vector<DMatch> &mvMatches12, vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21, const int mMaxIterations, int mSigma)
{
    vector<Match> match12;
    vectorDmatchToMatch(mvMatches12, match12);
    findHomography(mvKeys1, mvKeys2, match12, vbMatchesInliers, score, H21, mMaxIterations, mSigma);
}

void CommonAccessMethod::findHomography(vector<KeyPoint> &mvKeys1,
					vector<KeyPoint> &mvKeys2, vector<Match> &mvMatches12,
					vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21,
					const int mMaxIterations, int mSigma)
{
    //计算双方的匹配点个数
    const int N = mvMatches12.size();
    //将双方原有的点进行进行归一化处理
    vector<Point2f> vPn1, vPn2;
    Mat T1, T2;
    Normalize(mvKeys1, vPn1, T1);
    Normalize(mvKeys2, vPn2, T2);
    Mat T2inv = T2.inv();
    score = 0.0;
    vbMatchesInliers = vector<bool>(N, false);

    // 变量筛选  主要是使用8点法进行
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i; //计算处两个H矩阵  这两个人矩阵分别互为逆矩阵

    // 每次RANSAC的MatchesInliers与得分
    vector<bool> vbCurrentInliers(N, false);
    float currentScore;
    vector<vector<size_t>> mvSets;
    this->getRandomSet(N, mMaxIterations, mvSets);

    for (int it = 0; it < mMaxIterations; it++)
    {
	for (size_t j = 0; j < 8; j++)
	{
	    int idx = mvSets[it][j];
	    // vPn1i和vPn2i为匹配的特征点对的坐标
	    vPn1i[j] = vPn1[mvMatches12[idx].first];
	    vPn2i[j] = vPn2[mvMatches12[idx].second];
	}
	Mat Hn = computerHomography(vPn1i, vPn2i);
	// 恢复原始的均值和尺度
	H21i = T2inv * Hn * T1;
	H12i = H21i.inv();
	// 利用重投影误差为当次RANSAC的结果评分
	currentScore = CheckHomography(mvKeys1, mvKeys2, mvMatches12, H21i, H12i, vbCurrentInliers, mSigma);
	// 得到最优的vbMatchesInliers与score
	if (currentScore > score)
	{
	    H21 = H21i.clone();
	    vbMatchesInliers = vbCurrentInliers;
	    score = currentScore;
	}
    }
}

float CommonAccessMethod::CheckHomography(vector<KeyPoint> &mvKeys1, vector<KeyPoint> &mvKeys2, vector<Match> &mvMatches12, const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    // |h11 h12 h13|
    // |h21 h22 h23|
    // |h31 h32 h33|
    const float h11 = H21.at<float>(0, 0);
    const float h12 = H21.at<float>(0, 1);
    const float h13 = H21.at<float>(0, 2);
    const float h21 = H21.at<float>(1, 0);
    const float h22 = H21.at<float>(1, 1);
    const float h23 = H21.at<float>(1, 2);
    const float h31 = H21.at<float>(2, 0);
    const float h32 = H21.at<float>(2, 1);
    const float h33 = H21.at<float>(2, 2);

    // |h11inv h12inv h13inv|
    // |h21inv h22inv h23inv|
    // |h31inv h32inv h33inv|
    const float h11inv = H12.at<float>(0, 0);
    const float h12inv = H12.at<float>(0, 1);
    const float h13inv = H12.at<float>(0, 2);
    const float h21inv = H12.at<float>(1, 0);
    const float h22inv = H12.at<float>(1, 1);
    const float h23inv = H12.at<float>(1, 2);
    const float h31inv = H12.at<float>(2, 0);
    const float h32inv = H12.at<float>(2, 1);
    const float h33inv = H12.at<float>(2, 2);

    vbMatchesInliers.resize(N);

    float score = 0;

    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
    const float th = 5.991;

    //信息矩阵，方差平方的倒数
    const float invSigmaSquare = 1.0 / (sigma * sigma);

    // N对特征匹配点
    for (int i = 0; i < N; i++)
    {
	bool bIn = true;
	//获取每一个参数
	const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
	const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

	const float u1 = kp1.pt.x;
	const float v1 = kp1.pt.y;
	const float u2 = kp2.pt.x;
	const float v2 = kp2.pt.y;

	// Reprojection error in first image
	// x2in1 = H12*x2
	// 将图像2中的特征点单应到图像1中
	// |u1|   |h11inv h12inv h13inv||u2|
	// |v1| = |h21inv h22inv h23inv||v2|
	// |1 |   |h31inv h32inv h33inv||1 |
	const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
	const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
	const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

	// 计算重投影误差
	const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

	// 根据方差归一化误差
	const float chiSquare1 = squareDist1 * invSigmaSquare;

	if (chiSquare1 > th)
	    bIn = false;
	else
	    score += th - chiSquare1;

	// Reprojection error in second image
	// x1in2 = H21*x1
	// 将图像1中的特征点单应到图像2中
	const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
	const float u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
	const float v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

	const float squareDist2 = (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

	const float chiSquare2 = squareDist2 * invSigmaSquare;

	if (chiSquare2 > th)
	    bIn = false;
	else
	    score += th - chiSquare2;

	if (bIn)
	    vbMatchesInliers[i] = true;
	else
	    vbMatchesInliers[i] = false;
    }

    return score;
}

} /* namespace YOUSHEN_SLAM */
