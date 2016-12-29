#ifndef BINOCULAR_H
#define BINOCULAR_H
#include <opencv2/opencv.hpp>
#include "maker_binocular.h"
#include "RecTest.h"
#include "FileOperationTool.h"

namespace YOUSHEN_SLAM
{
class Binocular
{
  public:
    Binocular();
    virtual ~Binocular();
    YOUSHEN_SLAM::FileOperationTool fileTool;
    void getLeftAndRight(cv::Mat &left_image, cv::Mat &right_image);
    cv::Mat g_left_image;
    cv::Mat g_right_image;
    YOUSHEN_SLAM::RecTest* rec;
    float image_interval;  // image interval,
    float imu_interval[4]; // imu timestamp, time interval after last sample
    float acc[12];
    float gyro[12];
    void start();
    YOUSHEN_SLAM::makerbinocular m_makerbinocular;
};
}
#endif /* BINOCULAR_H */
