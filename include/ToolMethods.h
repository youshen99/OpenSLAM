#ifndef TOOLMETHODS_H
#define TOOLMETHODS_H
#include<opencv2/opencv.hpp>

namespace YOUSHEN_SLAM
{
   class ToolMethods{
      public:
        static void isImageToGrat(cv::Mat&image,cv::Mat&gray);

};
}

#endif /* TOOLMETHODS_H */