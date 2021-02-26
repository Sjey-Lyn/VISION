#include "opencv2/opencv.hpp"
#include <bits/stdc++.h>
#include "rp_kalman.h"
#include "object_predict.h"
#include <serialport.h>
using namespace cv;
using namespace std;
static main_information *main_info;
main_information::main_information(){
    drop_frame = 0;
    speed_kf.init_kalman_filter(0);
    accel_kf.init_kalman_filter(0);
    pkf.init_kalman_filter(0);
    ykf.init_kalman_filter(0);
}
void main_information::set_vdata_safe(VisionData &vdata, Mat &tvec)
{
    double distance;
    if(pre_frame.type || now_frame.type)
    {
        if(isnan(tvec.at<double>(1)))
        {
            speed_kf.init_kalman_filter(0);
            tvec = object_predict(pre_frame, now_frame, 1);
        }
        distance = sqrt(pow(tvec.at<double>(0),2)+pow(tvec.at<double>(1),2)+pow(tvec.at<double>(2),2));
        double correct_p = atan2(-tvec.at<double>(1), tvec.at<double>(2))*180/pi;
        pkf.correct_value(atan2(-tvec.at<double>(1), tvec.at<double>(2))*180/pi);
        if(isnan(correct_p))
        {
            vdata.pitch_angle.f = atan2(-tvec.at<double>(1), tvec.at<double>(2))*180/pi;
            pkf.init_kalman_filter(0);
        }
        else
        {
            vdata.pitch_angle.f = correct_p;
        }
        if(isnan(vdata.pitch_angle.f))
        {
            vdata.pitch_angle.f = 0.0;
        }
        double correct_y = atan2(tvec.at<double>(2), tvec.at<double>(0))*180/pi - 90;
        ykf.correct_value(atan2(tvec.at<double>(2), tvec.at<double>(0))*180/pi - 90);
        if(isnan(correct_y))
        {
            vdata.yaw_angle.f = atan2(tvec.at<double>(2), tvec.at<double>(0))*180/pi - 90;
            ykf.init_kalman_filter(0);
        }
        else
        {
            vdata.yaw_angle.f = correct_y;
        }
        if(isnan(vdata.yaw_angle.f))
        {
            vdata.yaw_angle.f = 0.0;
        }
        vdata.dis.f = distance;
        if(isnan(vdata.dis.f))
        {
            vdata.dis.f = -1;
        }
        vdata.isFindTarget = 1;
    }
    else
    {
        vdata.pitch_angle.f = 0;
        vdata.yaw_angle.f = 0;
        vdata.dis.f = 0;
        vdata.isFindTarget = 0;
    }
}
void set_maininfo(main_information *mf)
{
    main_info = mf;
}
//***********************帧信息类方法实现***********************//
// 构造函数
frame_information::frame_information()
{
    type = -1;
    tVec = (Mat_<double>(3,1) << 0, 0, 0);
    delta_T = 0.006;
}
// 获取该帧识别到的装甲板中心
Point2f frame_information::get_armoured_plate_center()
{
    return armoured_plate.center;
}
// 计算欧式距离函数

// 预测函数
// 参数：前后帧信息类，以及采用的模式
// 返回值：Mat<double>数组，第1个值是angel_yx，第2个值是angel_zx。
Mat object_predict(frame_information &pre_frame, frame_information &now_frame, int mode)
{
    double speed = 0;
    if(pre_frame.type == 0)
    {
        return now_frame.tVec;
    }
    else if(pre_frame.type == 1 && now_frame.type == 0)
    {
        speed = pre_frame.object_speed;
        now_frame.tVec = pre_frame.tVec;
    }
    else
    {
        double move_dist = norm(pre_frame.tVec,now_frame.tVec,NORM_L2);
        speed = move_dist/now_frame.delta_T;
    }
    if(now_frame.object_speed * pre_frame.object_speed < 0)
    {
        return now_frame.tVec;
    }
    if(speed < SPEED_LOW_BOUND)
    {
        return now_frame.tVec;
    }
    double x,y,z;
    double sin_xy, cos_xy;
    x = now_frame.tVec.at<double>(0,0);
    y = now_frame.tVec.at<double>(0,1);
    z = now_frame.tVec.at<double>(0,2);
    // 考虑小车沿x,y平面方向前行，并对预测结果进行校正
    if(mode == 1)
    {
        speed = main_info->speed_kf.correct_value(speed);
        double len = sqrt(pow(x,2)+pow(y,2));
        sin_xy = y/len;
        cos_xy = x/len;
        x += speed*cos_xy*PREDICT_ALPHA*DELAY_TIME;
        y += speed*sin_xy*PREDICT_BETA*DELAY_TIME;
    }
    // 考虑小车沿x,y平面方向前行，且考虑了车头自身的旋转，并对预测结果进行校正

//    cout<< x << "   " << y << "     " << z << endl;
    return (Mat_<double>(1, 3) << x, y, z);
}



// 插入新的速度到速度列表中
void insert_new_speed(double new_speed)
{
    main_info->speed_kf.correct_value(new_speed);
    if(main_info->object_speed.size() < MAX_VECTOR_LENGTH)
    {
        main_info->object_speed.push_back(new_speed);
    }
    else
    {
        vector<double>::iterator first = main_info->object_speed.begin();
        main_info->object_speed.erase(first);//删除第一个元素
        main_info->object_speed.push_back(new_speed);
    }
}
// 插入新的加速度到加速度列表中
void insert_new_accel(double new_accel)
{
    main_info->accel_kf.correct_value(new_accel);
    if(main_info->object_accel.size() < MAX_VECTOR_LENGTH)
    {
        main_info->object_accel.push_back(new_accel);
    }
    else
    {
        vector<double>::iterator first = main_info->object_accel.begin();
        main_info->object_accel.erase(first);//删除第一个元素
        main_info->object_accel.push_back(new_accel);
    }
}
// 计算向量列表平均值
double get_vector_average(vector<double> &vec)
{
    double sum = accumulate(std::begin(vec), std::end(vec), 0.0);
    return sum / vec.size(); //均值
}
// 基于接受数据列表来计算两帧间的绝对速度
int sub_sgn(double x, double y)
{
    if(x - y < 0)
    {
        return -1;
    }
    else if (x - y > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
double get_absolute_speed()
{
    double new_speed = 0;
#ifdef PREDICT_DEBUG
    cout << "***************************" << endl;
    cout << "now_frame's origin tvec"<<endl;
    cout << main_info->now_frame.tVec <<endl;
    cout << "***************************" << endl;
#endif
    if((fabs(main_info->now_frame.car_pitch_angel) < 0.0  || fabs(main_info->now_frame.car_yaw_angel) < 0.0))
    {
        // 辅助解方程组的变量定义
        double temp1 = tan(pi/180*(main_info->now_frame.car_yaw_angel));
        double temp3 = tan(-pi/180*main_info->now_frame.car_pitch_angel);
        double temp2 = temp3*temp1;
        double delta_x = HEAD_DISTANCE*HEAD_DISTANCE/sqrt(pow(temp1,2)+pow(temp2,2)+1)
        *sub_sgn(main_info->pre_frame.car_yaw_angel, 0);
        double delta_y = fabs(temp2 * delta_x)
        * sub_sgn(main_info->pre_frame.car_pitch_angel, 0);
        double delta_z = fabs(temp1 * delta_x);
//                *sub_sgn(main_info->now_frame.car_yaw_angel, main_info->pre_frame.car_yaw_angel) *
//                sub_sgn(main_info->now_frame.car_pitch_angel, main_info->pre_frame.car_pitch_angel);
        main_info->now_frame.tVec.at<double>(0,0) += delta_x;
        main_info->now_frame.tVec.at<double>(0,1) += delta_y;
        main_info->now_frame.tVec.at<double>(0,2) += delta_z;
    }
    double move_dist = norm(main_info->pre_frame.tVec,main_info->now_frame.tVec,NORM_L2);
    new_speed = move_dist/main_info->now_frame.delta_T;
#ifdef PREDICT_DEBUG
    cout << "---------------------------" << endl;
    cout << "pre_frame's correct tvec"<<endl;
    cout << main_info->pre_frame.tVec <<endl;
    cout << "now_frame's correct tvec"<<endl;
    cout << main_info->now_frame.tVec <<endl;
    cout << "---------------------------" << endl;
    cout << "move_distance" << endl;
    cout << move_dist <<endl;
    cout << "###########################" << endl;
#endif
    return new_speed;
}
void predict_reset()
{
    main_info->drop_frame = 0;
    main_info->speed_kf.init_kalman_filter(0);
    main_info->accel_kf.init_kalman_filter(0);
    main_info->object_speed.clear();
    main_info->object_accel.clear();
}
void rp_vision_predict(frame_information &pre_frame, frame_information &now_frame,
                        double &send_pitch, double send_yaw, int mode)
{
    // 掉帧处理
    if(now_frame.type == 0)
    {
        now_frame.tVec = (Mat_<double>(3,1) << 0, 0, 0);
        now_frame.object_speed = 0;
        main_info->drop_frame++;
        if(main_info->drop_frame > DROP_FRAME_BOUND)
        {
            predict_reset();
        }
        return;
    }
    main_info->drop_frame = 0;
    // 连续两帧没有跟丢才可以开始计算速度和加速度
    double new_speed = 0;
    if(main_info->pre_frame.type != 0)
    {
        new_speed = get_absolute_speed();
        main_info->now_frame.object_speed = new_speed;
        double new_accel = (main_info->now_frame.object_speed -
        main_info->pre_frame.object_speed)/now_frame.delta_T;
        insert_new_accel(new_accel);
    }
    // 速度死区，该部分不予预测
    if(fabs(new_speed) < PREDICT_SPEED_BOUND)
    {
        return;
    }
    insert_new_speed(new_speed);
    if(main_info->object_speed.size() < PREDICT_BEGIN)
    {
        return;
    }
    double object_average_speed = get_vector_average(main_info->object_speed);
    double object_average_accel = get_vector_average(main_info->object_accel);
    double x,y,z;
    double sin_xy, cos_xy;
    x = now_frame.tVec.at<double>(0,0);
    y = now_frame.tVec.at<double>(0,1);
    z = now_frame.tVec.at<double>(0,2);
    // 考虑小车沿x,y平面方向前行，并对预测结果进行校正
    if(mode == 1)
    {
        double len = sqrt(pow(x,2)+pow(y,2));
        // 以xy面为主平面
        sin_xy = y/len;
        cos_xy = x/len;
        // x = vt + 1/2*a*t^2
        double predict_distance = object_average_speed*DELAY_TIME +
        1/2 * object_average_accel * DELAY_TIME * DELAY_TIME;
        x += predict_distance*cos_xy*PREDICT_ALPHA;
        y += predict_distance*sin_xy*PREDICT_BETA;
    }
    send_pitch = atan2(-y, z)*180/pi;
    send_yaw = atan2(z, x)*180/pi - 90;
}
