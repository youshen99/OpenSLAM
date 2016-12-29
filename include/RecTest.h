#ifndef RECTEST_H
#define RECTEST_H
#include<opencv2/opencv.hpp>

namespace YOUSHEN_SLAM
{
  class RecTest{
      public:
      RecTest();
      virtual~RecTest();
      void MyCallback(cv::Mat&,cv::Mat&);
    };
    
}


#endif /* RECTEST_H */
