/*
 * @Author: youshen 
 * @Date: 2016-12-30 15:49:37 
 * @Last Modified by: youshen
 * @Last Modified time: 2016-12-30 16:21:23
 */
#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "Frame.h"

namespace YOUSHEN_SLAM
{
    
class Optimizer{
   public:
   //优化当前位姿状态
   void static PoseOptimization(Frame* frame);
};

}
#endif /* OPTIMIZER_H */
