#include"deal_drop_frame.h"


void drop_frame_ctrl::judge_drop_frame()
{
    if(lost_time > 0 && lost_time < LOST_OBJECT)
    {
        drop_frame = 1;
//        drop_frame_recorrect();
    }
    else
    {
        drop_frame = 0;
    }

}


void drop_frame_ctrl::insert_values(int flag,double pitch, double yaw, double distance)
{
    if(flag)
    {
        if(lost_time != 0)
        {
                cout<<"掉帧数"<<lost_time<<endl;
        }

        if(lost_time > 0)
        {
            pitch_vec.clear();
            yaw_vec.clear();
        }
        lost_time = 0;



//针对装甲板切换
        if(yaw_vec.size() > 0 && yaw_vec.back() > 0 && yaw< 0)
        {
            yaw_vec.clear();
        }
        else if(yaw_vec.size() > 0 && yaw_vec.back() < 0 && yaw > 0)
        {
            yaw_vec.clear();
        }

        if(pitch_vec.size() > 0 && pitch_vec.back() > 0 && yaw< 0)
        {
            pitch_vec.clear();
        }
        else if(pitch_vec.size() > 0 && pitch_vec.back() < 0 && yaw > 0)
        {
            pitch_vec.clear();
        }




        if(yaw_vec.size() < VECTOR_MAX)
        {
            yaw_vec.push_back(yaw);

        }
        else
        {
            vector<double>::iterator first = yaw_vec.begin();
            yaw_vec.erase(first);
            yaw_vec.push_back(yaw);
        }


        if(pitch_vec.size() < VECTOR_MAX)
        {
            pitch_vec.push_back(pitch);
        }
        else
        {
            vector<double>::iterator first = pitch_vec.begin();
            pitch_vec.erase(first);
            pitch_vec.push_back( pitch);
        }

        last_distance = distance;


    }
    else
    {
        lost_time++;
    }
}



void drop_frame_ctrl::get_aspeed()
{
    double yaw_sum1 = 0, yaw_sum2 = 0;
    double pit_sum1 = 0, pit_sum2 = 0;


    if(yaw_vec.size()%2 == 1 && yaw_vec.size() > 1)
    {
        yaw_vec.erase(yaw_vec.begin());

        for(int i = 0 ; i < yaw_vec.size() / 2; i++)
        {
            yaw_sum1 += yaw_vec.at(i);
        }

        for(int i = yaw_vec.size() / 2 ; i < yaw_vec.size() ; i++)
        {
            yaw_sum2 += yaw_vec.at(i);
        }
        y_a_speed = (yaw_sum2 - yaw_sum1)/(yaw_vec.size() / 2 )/(yaw_vec.size() / 2);
        y_a_speed = y_a_speed > 0.5 ? 0.5 : y_a_speed;
    }

    else if(yaw_vec.size()%2 == 0 && yaw_vec.size() > 1)
    {
        for(int i = 0 ; i < yaw_vec.size() / 2; i++)
        {
            yaw_sum1 += yaw_vec.at(i);
        }

        for(int i = yaw_vec.size() / 2 ; i < yaw_vec.size() ; i++)
        {
            yaw_sum2 += yaw_vec.at(i);
        }

        y_a_speed = (yaw_sum2 - yaw_sum1)/ (yaw_vec.size() / 2 )/(yaw_vec.size() / 2);
        y_a_speed = y_a_speed > 0.5 ? 0.5 : y_a_speed;
    }

    else if(yaw_vec.size()  == 1)
    {
        y_a_speed = 0;
    }


    if(pitch_vec.size()%2 == 1 && pitch_vec.size() > 1)
    {
        pitch_vec.erase(pitch_vec.begin());

        for(int i = 0 ; i < pitch_vec.size() / 2; i++)
        {
            pit_sum1 += pitch_vec.at(i);
        }

        for(int i = pitch_vec.size() / 2 ; i < pitch_vec.size() ; i++)
        {
            pit_sum2 += pitch_vec.at(i);
        }
        p_a_speed = (pit_sum2 - pit_sum1)/ (pitch_vec.size() / 2);
        p_a_speed = p_a_speed > 0.5 ? 0.5 : p_a_speed;

   }
    else if(pitch_vec.size()%2 == 0 && pitch_vec.size() > 1)
    {
        for(int i = 0 ; i < pitch_vec.size() / 2; i++)
        {
            pit_sum1 += pitch_vec.at(i);
        }

        for(int i = pitch_vec.size() / 2 ; i < pitch_vec.size() ; i++)
        {
            pit_sum2 += pitch_vec.at(i);
        }
        p_a_speed = (pit_sum2 - pit_sum1)/ (pitch_vec.size() / 2);
        p_a_speed = p_a_speed > 0.5 ? 0.5 : p_a_speed;
    }
    else if(pitch_vec.size() == 1)
    {
        p_a_speed = 0;
    }



}


void drop_frame_ctrl::drop_frame_recorrect()
{
    if(yaw_vec.size() > 0 && pitch_vec.size() >0)
    {
        drop_yaw = yaw_vec.back() + Alpha *y_a_speed * lost_time;
        drop_pitch = pitch_vec.back() + Alpha *p_a_speed *lost_time;
        drop_distance = last_distance;
    }

}
