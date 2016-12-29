#include "Binocular.h"

namespace YOUSHEN_SLAM
{

Binocular::Binocular(){

};
Binocular::~Binocular()
{
}

void Binocular::start()
{
    this->g_left_image = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
    this->g_right_image = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
    cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));
    std::vector<cv::Mat> left_imagelist;
    std::vector<cv::Mat> right_imagelist;
    std::vector<double> timestep;
    {
        double t = 0;
        //获取上一个时间搓
        double lasttime = 0;
        while (1)
        {
            if (!m_makerbinocular.get_frame(left_image, right_image, acc, gyro, image_interval, imu_interval))
                continue;

            if (m_makerbinocular.new_frame_arrived())
            {
                cv::imshow("left image", left_image); //左摄像头
                cv::waitKey(1);
                cv::imshow("right image", right_image); //右摄像头
                cv::waitKey(1);
                //视频录制
                //  cv::imwrite("/home/glodon/imageShuang/1.jpg",left_image);
                left_imagelist.push_back(left_image.clone());
                right_imagelist.push_back(right_image.clone());
                if (lasttime == 0)
                {
                    lasttime = (double)cvGetTickCount();
                    timestep.push_back(0);
                }
                else
                {
                    double newtime = (double)cvGetTickCount();
                    timestep.push_back((newtime - lasttime) / (cvGetTickFrequency() * 1000));
                }
            }

            //  usleep(10000);
            char key = cv::waitKey(10);
            //    std::cout<<(int)key<<std::endl;
            if (key == 27)
            {
                std::cout << "----start------" << std::endl;
                std::cout << "照片共 " << left_imagelist.size() << " 张" << std::endl;
                for (size_t i = 0; i < left_imagelist.size(); i++)
                {

                    char buf[80];
                    std::sprintf(buf, "/home/glodon/imageShuang/left_image/%d.jpg", (int)i);
                    cv::imwrite(buf, left_imagelist[i]);

                    char buf2[80];
                    std::sprintf(buf2, "/home/glodon/imageShuang/right_image/%d.jpg", (int)i);
                    cv::imwrite(buf2, right_imagelist[i]);
                }
                //存时间
                fileTool.WriteFileByRowList(timestep, "/home/glodon/imageShuang/time.txt", std::ios::trunc);

                std::cout << "----end------" << std::endl;

                break;
            }
        }
    }
}
void Binocular::getLeftAndRight(cv::Mat &left_image, cv::Mat &right_image)
{
    left_image = this->g_left_image.clone();
    right_image = this->g_right_image.clone();
}
}