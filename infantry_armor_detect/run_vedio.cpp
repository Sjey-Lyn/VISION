//#define SHIPING
#ifdef SHIPING
#include<armordetection.h>
#include<numpredict.h>
#include<fstream>
int main()
{
    double t = (double)getTickCount();
    Mat frame;
    Numpredict num_pre;
    int index = 0;          // 记录帧数
    queue<Rect> last_roi;   //存放前几帧的roi
//    queue<Rect> frame_information;   //卡尔曼预处需要的信息
    last_roi.push(Rect(0,0, frame_W, frame_H));  // 第一帧roi是整张图片
//    frame_info init_frame_information;
//    init_frame_information.flag = 0;
//    frame_information.push(init_frame_information); // 初始化一下，第一帧的前一帧没有目标，flag为0
    double roi_scale=0.2;   // roi相对于旋转矩形的放大比例
    int loss_frame = 0;      // 掉帧系数
//    Mat kalman_result;       // 卡尔曼预测结果



    #define DEAL_DIAOZHEN
    #define VECTOR_MAX 20
    #define LOST_OBJECT 10
    int lost_time = 0;
    vector<double>yaw_vec;
    vector<double>pitch_vec;

    double send_yaw_angle ;
    ofstream send_yaw("send_yaw.txt");
    ofstream yaw("yaw.txt");
    ofstream a_speed_re("a_speed_yaw");
    Mat cam = (Mat_<double>(3, 3) << 1733.6, -5.3509, 656.7580,
                                     0.0000000000000000, 1729.8, 167.6416,
                                     0.0000000000000000, 0.0000000000000000, 1.0000000000000000);
    // 畸变系数
    Mat dis = (Mat_<double>(5, 1) <<-0.0593, 0.2841, -0.0032,
                                    0.0080, 0.0000000000000000);



    VideoCapture capture;
    capture.open("/home/lyn/Desktop/szu_armor_detect/1.avi");
    long totalFrameNumber=capture.get(CV_CAP_PROP_FRAME_COUNT);//获取视频的总帧数
    cout << "整个视频共" << totalFrameNumber << "帧" << endl;
    long frame_index =700;
    bool success = capture.set(CV_CAP_PROP_POS_FRAMES, frame_index);
    if(success)cout<<"success"<<endl;
    if(!capture.isOpened())
    {
        cout<<"video open failed "<<endl;
    }


    //0:按一下，动一下 ,1: 连续播放
    int iscontinue=0;

    while(1)
    {
        index++;
        cout<<"------------------------------------------------------------------> "<<index<<endl;
        capture>>frame;
//        namedWindow("frame",2);
//        imshow("frame",frame);
//        cout<<111111111111111<<endl;
        // 构造一个装甲板检测对象，需要: 检测的图片， 上一帧的roi， 掉帧系数
        Armordetection armordetection = Armordetection(frame, last_roi.back(), loss_frame);
//        cout<<22222222222222222<<endl;
        // 开始检测装甲板
//        void begin_to_detect(Mat c, Mat d, double roi_scale, double armorH, double armorW,
//                             bool use_roi, int color_detection_mode, bool show_mask,
//                             bool show_light_filter_result, bool show_match_result,
//                             bool get_num_image, int num_img_size, bool show_num_img)


        armordetection.begin_to_detect(cam, dis,    //Mat c, Mat d ------>相机内参, 畸变系数
                                       roi_scale,   //double roi_scale -----> roi相对于旋转矩形的放大比例
                                       armor_h,armor_w, //double armorH, double armorW -----> 检测的装甲板的实际高，宽
                                       1, //bool use_roi -----> 是否使用roi，建议调试时不使用，以观察算法的准确性，比赛时使用，以提高检测速度
                                       1, //int color_detection_mode 图像预处理提取二值图的模式，目前只有模式1，后期再补充
                                       0,//bool show_mask ------> 是否展示图像预处理效果
                                       0, 0,//bool show_light_filter_result, bool show_match_result ----> 是否展示筛选灯条，匹配灯条的效果
                                       0, // bool get_num_image -----> 是否提取装甲板上的数字, 如果否，则num_img_size, show_num_img随便给
                                       100, 0,// int num_img_size, bool show_num_img ---> 提取装甲板上的数字的图像大小， 以及是否展示提取的数字
                                       num_pre);//num_pred instance
        // 展示检测

        armordetection.show_img();
//        cout<<44444444444444444444444444<<endl;


        //这里记录前10帧的roi信息
        if ( last_roi.size() >= 10)
        {
            last_roi.pop();
        }
        last_roi.push(armordetection._next_roi);

        // 计算掉帧系数
        if(armordetection._flag == 1)
        {
            loss_frame = 0;
        }
        else
        {
            loss_frame++;
        }





        cout<<"flag"<<armordetection._flag<<endl;




        if(armordetection._flag)
        {
            lost_time = 0;
            if(yaw_vec.size() < VECTOR_MAX)
            {
                yaw_vec.push_back(armordetection.yaw_angle);

            }
            else
            {
                vector<double>::iterator first = yaw_vec.begin();
                yaw_vec.erase(first);
                yaw_vec.push_back(armordetection.yaw_angle);
            }


//            if(pitch_vec.size() < VECTOR_MAX)
//            {
//                pitch_vec.push_back(armordetection.pitch_angle);
//            }
//            else
//            {
//                vector<double>::iterator first = pitch_vec.begin();
//                pitch_vec.erase(first);
//                pitch_vec.push_back( armordetection.pitch_angle);
//            }



        }
        else
        {
            lost_time++;
        }


        if( lost_time > 0 && lost_time < LOST_OBJECT)
        {
            double yaw_sum1=0, yaw_sum2=0;
            double pit_sum1=0, pit_sum2=0;
//            vdata.dis.f = last_dantance;
//            vdata.isFindTarget = 1 ;
//            vdata.buff_change_four=0;
//            vdata.anti_top=0;
//            vdata.anti_top_change_armor = 0;
//            vdata.nearFace = 0;
//            vdata.isfindDafu= 0;


            if(yaw_vec.size()%2 == 1 && yaw_vec.size() > 10)
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

                send_yaw_angle = yaw_vec.back() + (yaw_sum2 - yaw_sum1)/ (yaw_vec.size() / 2 )/(yaw_vec.size() / 2) * lost_time ;
//                vdata.yaw_angle.f = send_yaw_angle;
                double a_speed = (yaw_sum2 - yaw_sum1)/ (yaw_vec.size() / 2 )/(yaw_vec.size() / 2);
                a_speed_re << endl << a_speed ;
                cout<< a_speed<< endl;

            }

            else if(yaw_vec.size()%2 != 1 && yaw_vec.size() >10)
            {
                for(int i = 0 ; i < yaw_vec.size() / 2; i++)
                {
                    yaw_sum1 += yaw_vec.at(i);
                }

                for(int i = yaw_vec.size() / 2 ; i < yaw_vec.size() ; i++)
                {
                    yaw_sum2 += yaw_vec.at(i);
                }

                send_yaw_angle = yaw_vec.back() + (yaw_sum2 - yaw_sum1)/ (yaw_vec.size() / 2) /(yaw_vec.size() / 2) * lost_time ;
                double a_speed = (yaw_sum2 - yaw_sum1)/ (yaw_vec.size() / 2 )/(yaw_vec.size() / 2);
                a_speed_re << endl << a_speed ;
                cout<< a_speed<< endl;
            }
        }
        else
        {
            send_yaw_angle = armordetection.yaw_angle;
        }



        send_yaw <<endl<< send_yaw_angle;
        yaw << endl << armordetection.yaw_angle;





//            waitKey(1);
        int comm= (int)waitKey(30);
        if(comm=='s')
        {
            iscontinue=0;
        }

        if(iscontinue==1)
        {
            waitKey(1);
            int delay = 30;
            if(delay >= 0 && waitKey(delay) >= 32) waitKey(1);
        }
        else if(iscontinue==0)
        {
            int comm2=waitKey(0);
            if(comm2=='c')
                iscontinue=1;
            int delay = 30;
            if(delay >= 0 && waitKey(delay) >= 32) waitKey(0);
        }


    }

    capture.release();//释放视频内存
    return 0;

    send_yaw.close();
    yaw.close();
    a_speed_re.close();

}




#endif
