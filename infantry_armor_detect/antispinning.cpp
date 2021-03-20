#include "antispinning.h"
// 用vector模拟队列进出
#include <math.h>
void insert_newvalue(vector<double> &vec, double value)
{
    if(vec.size() < QUEUE_MAX)
    {
        vec.push_back(value);
    }
    else
    {
        vector<double>::iterator first = vec.begin();
        vec.erase(first);//删除第一个元素
        vec.push_back(value);
    }
}
void insert_newvalue(vector<int> &vec, int value)
{
    if(vec.size() < QUEUE_MAX)
    {
        vec.push_back(value);
    }
    else
    {
        vector<int>::iterator first = vec.begin();
        vec.erase(first);//删除第一个元素
        vec.push_back(value);
    }
}
void anti_spinning_controller::insert_values(double pitch, double yaw, double distance)
{
    if(fabs(yaw)>35 || isnan(yaw)){
        if(yaw_vec.size()>0)
        {
            yaw = yaw_vec[(int)yaw_vec.size()-1]/2;
        }
        else
        {
            yaw=0;
            pitch=0;
            distance=0;
            return;
        }
    }
    if(fabs(pitch)>35 || isnan(pitch)){
        if(pitch_vec.size()>0)
        {
            pitch = pitch_vec[(int)pitch_vec.size()-1]/2;
        }
        else
        {
            yaw=0;
            pitch=0;
            distance=0;
            return;
        }
    }
    if(isnan(distance)){
        if(dis_vec.size()>0)
        {
            distance = dis_vec[(int)dis_vec.size()-1];
        }
        else
        {
            yaw=0;
            pitch=0;
            distance=0;
            return;
        }
    }
    insert_newvalue(dis_vec, distance);
    insert_newvalue(yaw_vec, yaw);
    insert_newvalue(pitch_vec, pitch);
}

void anti_spinning_controller::judge_is_spinning(int now_state, int pre_state, double distance, double yaw)
{
   double pre_yaw_delta = 0, yaw_delta = 0;
    if(now_state == 0)
    {
        drop_frame_param++;
        if(drop_frame_param>DROP_FRAME_PARAM_BOUND)
        {
            is_spinning = false;
            spinning_param = 0;
        }
        return;
    }
    drop_frame_param = 0;
    if(pre_state == 0)
    {
        insert_newvalue(status_vec,1);
        // 计算掉帧前的物距
        double pre_dis = 0;
        if(dis_vec.size()>0)
        {
            pre_dis = dis_vec[dis_vec.size() - 1];
        }
        // 计算物体掉帧前后的位移，一般陀螺状态下，这段位移不是很大，但是会相对确定。
        double move_distance = fabs(distance - pre_dis);
#ifdef DEBUG_SPIN_COUT
        cout<<"*********************************************"<<endl;
        cout<<"物体掉帧前后的位移: "<< move_distance <<endl;
        cout<<"识别为陀螺状态的位移误差上限: "<< SPINNING_DISTENCE_UP_BOUND <<endl;
        cout<<"识别为陀螺状态的位移误差下限: "<< SPINNING_DISTENCE_LOW_BOUND <<endl;
#endif
/*
下一个条件是为了将陀螺状态与闪烁掉帧情况彻底区分开来
通过yaw轴差分，锁定陀螺方向，因为yaw轴的差分在转动到另一个装甲板时是异号。
而闪烁掉帧情况下，如果非陀螺仪状态，那么yaw轴的差分一般都是同号的。
一般认为陀螺方向一段时间内是不变的。
要变陀螺方向的话，需要减速到停，这个过程够退出陀螺状态的识别。
因此，进行新方向的陀螺测试时，前面的测试参数已经清空，不会与上一次冲突。
*/
        //
        if(yaw_vec.size()>=2)
        {
            pre_yaw_delta = yaw_vec[yaw_vec.size() - 1] - yaw_vec[yaw_vec.size() - 2];
        }
        if(yaw_vec.size()>0)
        {
            yaw_delta = yaw - yaw_vec[yaw_vec.size() - 1];
        }
        double now_spin_direction = MathUtils_SignBit(pre_yaw_delta) - MathUtils_SignBit(yaw_delta);
#ifdef DEBUG_SPIN_COUT
        cout<<"now_spin_direction = "<< now_spin_direction<<endl;
        cout<<"*********************************************"<<endl;
        cout<<"pre_yaw_delta"<<pre_yaw_delta<<endl;
        cout<<"yaw_delta"<<yaw_delta<<endl;
#endif
        if(spin_direction == 0)
        {
            spin_direction = now_spin_direction;
        }
        ///
        if(move_distance>SPINNING_DISTENCE_LOW_BOUND &&
           move_distance<SPINNING_DISTENCE_UP_BOUND &&
           fabs(pre_yaw_delta) < YAW_DELTA_LOW_BOUND && fabs(yaw_delta) > YAW_DELATA_UP_BOUND
                )
        {
            if(now_spin_direction != spin_direction)
            {
                spin_direction = now_spin_direction;
            }
            spinning_param++;
            continues_judge_param = 0;
#ifdef DEBUG_SPIN_COUT
        cout<<"陀螺系数加一了!"<<endl;
        cout<<"陀螺系数"<<spinning_param<<endl;
#endif
        }
        else
        {

            // spinning_param--;
            continues_judge_param++;
        }
#ifdef DEBUG_SPIN_COUT
        cout<<"is_spinning = "<<is_spinning<<endl;
        cout<<"||||||||||||||||||||||||||||||"<<endl;
#endif
//        if(is_spinning&&spinning_pitch<0.01&&spinning_yaw<0.01&&spinning_process_param>100)
//        {
//            if(MathUtils_SignBit(pre_yaw_delta) == MathUtils_SignBit(yaw_delta))
//            {
//                spinning_param--;
//            }
//        }
    }
    else
    {
        insert_newvalue(status_vec,0);
        continues_judge_param++;
    }
    if(spinning_param >= SPINNING_BOUND)
    {
        is_spinning = true;
    }
    if(continues_judge_param > CONTINUES_JUDGE_PARAM_BOUND || drop_frame_param > DROP_FRAME_PARAM_BOUND)
    {
        spinning_param = 0;
        spin_direction = 0;
        is_spinning = false;
    }
}

double anti_spinning_controller::get_spinning_information(int flag)
{
    double res = 0.0, head = 0.0, tail = 0.0;
    if(flag == 0)
    {
        for(int i=(int)pitch_vec.size() - 1, j = 0; i >= 0 && j < 10; i--, j++)
        {
            res +=pitch_vec[i];
        }
        res = res/10.0;
    }
    else if(flag == 1)
    {
        int temp = 0;
        for(int i=(int)yaw_vec.size()-1; i>=0; i--)
        {
            if(status_vec[i])
            {
                if(temp%2==0)
                {
                    if(temp != 0)
                    {
                        head = (head*7 + yaw_vec[i]*3)/10;
                    }
                    else
                    {
                        head = yaw_vec[i];
                    }
                }
                else
                {
                    if(temp != 1)
                    {
                        tail = (tail*7 + yaw_vec[i]*3)/10;
                    }
                    else
                    {
                        tail = yaw_vec[i];
                    }
                }
                temp++;
            }
        }
        res = (head + tail) / 2;
    }
    else if(flag=2)
    {
        for(int i=(int)dis_vec.size() - 1, j = 0; i >= 0 && j < 10; i--, j++)
        {
            res +=dis_vec[i];
        }
        res = res/10.0;
    }
    return res;
}

void anti_spinning_controller::spinning_process(int mode)
{
    if(mode == 0)
    {
        if(spinning_process_param%REMAIN_FRAMES == 0)
        {
            spinning_distance = get_spinning_information(2) ;
            // 添加补偿角
            spinning_pitch = get_spinning_information(0) + 4;
            spinning_yaw = get_spinning_information(1);//-spin_direction*2;
            cout<<"spinning_yaw="<<spinning_yaw<<endl;
            spinning_process_param++;
        }
        else
        {
            cout<<"spinning_pitch="<<spinning_pitch<<endl;
            spinning_pitch = spinning_pitch/1.2;
            spinning_pitch = fabs(spinning_pitch)<0.001?spinning_pitch:0;
            spinning_yaw = spinning_yaw/1.2;
            spinning_yaw = fabs(spinning_yaw)<0.001?spinning_yaw:0;
            spinning_distance = get_spinning_information(2);
            spinning_process_param++;
        }
    }
}
