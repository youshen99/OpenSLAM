#include "ToolMethods.h"
namespace YOUSHEN_SLAM
{
   void ToolMethods::isImageToGrat(cv::Mat&image,cv::Mat&gray){
	if(image.type()==CV_8UC1){
       gray =image;
	}else{
       cvtColor(image,gray,CV_BGR2GRAY);
	}
}
/**
通过文件获取相机的内参和畸变函数
*/
void ToolMethods::getSettingFile(std::string settingPath,cv::Mat&mk,cv::Mat&mDistCoef){
 cv::FileStorage fSettings(settingPath.c_str(), cv::FileStorage::READ);
//路径无效的情况
if(!fSettings.isOpened()){
      std::cout << "Failed to open settings file at: " << settingPath << std::endl;
      exit(-1);
}
//读取对应的参数文件
 float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    //     |fx  0   cx|
    // K = |0   fy  cy|
    //     |0   0   1 |
    //构建相机内参矩阵
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mk); 
     // 图像矫正系数
    //获取图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);
}

//前者的長度比後者要大
void ToolMethods::computerMatch2(std::vector<cv::DMatch>&dMatch12,std::vector<cv::DMatch>&dMatch22,std::vector<cv::DMatch>&dMatchLaster2){
     for (size_t i = 0; i < dMatch12.size(); i++)
     {
        for (size_t j = 0; j < dMatch22.size(); j++)
       {//判断对应的前后只是否是一样的
            if(dMatch12[i].queryIdx==dMatch22[j].trainIdx){
              if(dMatch22[j].queryIdx==dMatch12[i].trainIdx){
                dMatchLaster2.push_back(dMatch12[i]);       
                 }
               }
         }
     }
   }

void ToolMethods::computerMatchPoint(std::vector<cv::DMatch>&dMatch1,std::vector<cv::DMatch>&dMatch2,std::vector<cv::DMatch>&dMatchLaster){
     //计算输入的特征点的长度
     if(dMatch1.size()<dMatch2.size()){
       
         computerMatch2(dMatch2,dMatch1,dMatchLaster);
     }else{
         computerMatch2(dMatch1,dMatch2,dMatchLaster);
     }
}

 bool ToolMethods::computerParallax(const cv::Mat &rotation_mat,std::vector<float> &angleZXY){
     //判断是否是旋转矩阵 
     if(rotation_mat.rows!=3||rotation_mat.cols!=3){
        return false;
     }
     //Mat 转 matrix
     Eigen::Matrix3d roat_matrix = ToolMethods::Mat2Matrix3d(rotation_mat);
     //计算对应的旋转矩阵 
      Eigen::Vector3d euler_angles = roat_matrix.eulerAngles ( 2,1,0 );
      std::cout<<euler_angles<<std::endl;//******************80
      angleZXY.push_back(euler_angles(0,0));
      angleZXY.push_back(euler_angles(1,0));
      angleZXY.push_back( euler_angles(2,0));
 }
 Eigen::Matrix3d ToolMethods::Mat2Matrix3d(const cv::Mat& mat){
       Eigen::Matrix3d matrix;
       for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){ 
                matrix(i,j)=mat.at<float>(i,j);
          }
       }
       return matrix;
 }

}