#include "opencv2/opencv.hpp"
#include <bits/stdc++.h>
#include "rp_kalman.h"
#include <serialport.h>
using namespace cv;
using namespace std;

#ifndef OBJECT_PREDICT_H
#define OBJECT_PREDICT_H

#define PREDICT_DEBUG


#define pi 3.14159265
/****** 预测算法 ******/
// 传输数据给电控过程以及打弹过程中延迟时间
#define DELAY_TIME 0.5
#define SPEED_LOW_BOUND 0.01
// 枪口的半径
#define CARHEAD_RADIUS 0.1
// 对世界坐标x、y、z的矫正系数
#define PREDICT_ALPHA 1
#define PREDICT_BETA 1
#define PREDICT_LAMDA 1

#define PREDICT_BEGIN 5
#define MAX_VECTOR_LENGTH 10
// 速度绝对值大于等于0.2才可以进行预测
#define PREDICT_SPEED_BOUND 300
#define HEAD_DISTANCE 0.4
#define DROP_FRAME_BOUND 15


struct frame_information
{
    int type;
    RotatedRect armoured_plate;
    Mat tVec;
    double delta_T;
    double object_speed;
    double car_yaw_angel;
    double car_pitch_angel;
    double yaw_angel_speed;
    double pitch_angel_speed;
    double carhead_angel_speed;
    frame_information();
    Point2f get_armoured_plate_center();
};

struct main_information{
    // 前后帧信息变量
    int drop_frame;
    frame_information pre_frame;
    frame_information now_frame;

    kalman_filter speed_kf;
    kalman_filter accel_kf;
    kalman_filter pkf;
    kalman_filter ykf;

    vector<double> object_speed;
    vector<double> object_accel;

    main_information();
    void set_vdata_safe(VisionData &vdata, Mat &tvec);
};
void set_maininfo(main_information *mf);
Mat object_predict(frame_information &pre_frame, frame_information &now_frame, int mode);
int sub_sgn(double x, double y);
void rp_vision_predict(frame_information &pre_frame, frame_information &now_frame,
                        double &send_pitch, double send_yaw, int mode = 1);
#endif // OBJECT_PREDICT_H
