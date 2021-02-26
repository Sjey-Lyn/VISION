#ifndef NUMPREDICT_H
#define NUMPREDICT_H
#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<queue>
#include<math.h>
#include<algorithm>

using namespace cv;
using namespace std;
//此类用于数字识别,使用lenet，
//在启动时初始化，使用时调用predict传入经过仿射变换的数字图片，返回一个代表类的数字。
//类0-4对应数字1-5、 类5：雷达、 类6：哨兵  、类7：基地 、类8：非识别类
class Numpredict{
public:
    Numpredict();
    int num_predict(Mat& img);//对传入的图片进行预测
    void gamma_trans(Mat &img_num,Mat &img_gamma);//gamma transform
    float auto_exposure(float cur_exposure_time,Mat &img_gamma,int enemy_color);
    int predict_project(Mat& img_num, bool is_the_same_object);//通过投票判断数字类型，is_the_same_object=true则本次传入和上次传入的图片来自同一目标
private:
    cv::dnn::Net net;
    float gamma;
    int frames ;   //同一个目标传入的图片数目
    int fixed_label;
    int possibleNum[9];  //当对同一目标预测得到的类别出现的次数.
    void getMaxClass(const Mat& probBlob, int* classId, double* classProb);

};


#endif // NUMPREDICT_H
