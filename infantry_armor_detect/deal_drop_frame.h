#ifndef DEAL_DROP_FRAME_H
#define DEAL_DROP_FRAME_H
#include<iostream>
#include<vector>
using namespace std;

#define VECTOR_MAX 20
#define LOST_OBJECT 20
#define Alpha 0.5

struct drop_frame_ctrl
{
    vector<double> yaw_vec;
    vector<double> pitch_vec;
    double last_distance = 0;

    bool drop_frame = 0;
    int lost_time = 0;
    double y_a_speed = 0;
    double p_a_speed = 0;

    double drop_yaw = 0;
    double drop_pitch = 0;
    double drop_distance  = 0;

    void judge_drop_frame();
    void insert_values(int flag, double pitch, double yaw, double distance);
    void get_aspeed();
    void drop_frame_recorrect();


};




#endif // DEAL_DROP_FRAME_H


