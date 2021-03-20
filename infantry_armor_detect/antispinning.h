#ifndef ANTISPINNING_H
#define ANTISPINNING_H
#include<iostream>
#include<vector>
#include<cmath>
using namespace std;
#define MathUtils_SignBit(x) \
    (((signed char*) &x)[sizeof(x) - 1] >> 7 | 1)
// 与反陀螺有关的阈值
#define SPINNING_BOUND 3 //陀螺系数的上界,要识别出陀螺状态3次才认为是陀螺
#define CONTINUES_JUDGE_PARAM_BOUND 200 //持续识别次数,在120帧内必须识别出陀螺状态,陀螺系数+1,否则陀螺系数置0
#define DROP_FRAME_PARAM_BOUND 20        //掉帧系数上界,陀螺期间发生掉帧的间隔不能超过200,否则陀螺系数也会置0
#define SPINNING_DISTENCE_UP_BOUND 700
#define SPINNING_DISTENCE_LOW_BOUND 10
#define REMAIN_FRAMES 1000   //每隔一定帧数更新pitch,yaw,取适中,不至于定死,又可以定住一段时间
#define QUEUE_MAX 1000
#define YAW_DELTA_LOW_BOUND 0.5
#define YAW_DELATA_UP_BOUND 3

#define DEBUG_SPIN_COUT
struct anti_spinning_controller{
    // 与反陀螺有关的变量
    bool is_spinning = false;
    int spinning_param = 0;
    int continues_judge_param = 0;
    int drop_frame_param = 0;
    int spinning_process_param = 0;
    // 用于判断陀螺是向视角左转还是右转，左为-2，右为2
    int spin_direction = 1;
    vector<double> yaw_vec;
    vector<double> pitch_vec;
    vector<double> dis_vec;
    vector<int> status_vec;

    double spinning_pitch;
    double spinning_yaw;
    double spinning_distance;
    // 识别陀螺状态的函数
    void insert_values(double pitch, double yaw, double distance);
    void judge_is_spinning(int now_state, int pre_state,  double distance, double yaw);
    double get_spinning_information(int flag);
    void spinning_process(int mode);
};


#endif // ANTISPINNING_H
