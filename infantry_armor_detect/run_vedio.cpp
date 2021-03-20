//#define SHIPING
#ifdef SHIPING
#include<armordetection.h>
#include<antispinning.h>
#include"deal_drop_frame.h"
#include<numpredict.h>
#include<fstream>

int main()
{
    double t = (double)getTickCount();
    Mat frame;
    Numpredict num_pre;
    int index = 0;          // 记录帧数
    queue<Rect> last_roi;   //存放前几帧的roi
    last_roi.push(Rect(0,0, frame_W, frame_H));  // 第一帧roi是整张图片
    double roi_scale= 1.5;   // roi相对于旋转矩形的放大比例
    int loss_frame = 0;      // 掉帧系数
////////////反陀螺///////////
    anti_spinning_controller asc;
/////////////掉帧//////////
    drop_frame_ctrl dfc;
    int pre_flag=0;
    double pre_yaw_angle = 0;


    // 相机内参
    Mat cam = (Mat_<double>(3, 3) << 2000, 0, 700,
                                     0.0000000000000000, 2000, 265.3870,
                                     0.0000000000000000, 0.0000000000000000, 1.0000000000000000);
    // 畸变系数
    Mat dis = (Mat_<double>(5, 1) << 0.0576, 0.4540, -0.0014,
                                    -0.0118, 0.0000000000000000);



    VideoCapture capture;
    capture.open("/home/nuc/Desktop/build-infantry_armor_detect-Desktop_Qt_5_9_1_GCC_64bit-Debug/Fri Mar 19 03:45:46 2021.avi");
    long totalFrameNumber=capture.get(CV_CAP_PROP_FRAME_COUNT);//获取视频的总帧数
    cout << "整个视频共" << totalFrameNumber << "帧" << endl;
    long frame_index =0;
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
        // 构造一个装甲板检测对象，需要: 检测的图片， 上一帧的roi， 掉帧系数
        Armordetection armordetection = Armordetection(frame, last_roi.back(), loss_frame);
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



//        dfc.insert_values(armordetection._flag,armordetection.pitch_angle, armordetection.yaw_angle, armordetection.distance);
//        dfc.get_aspeed();

        // 嵌入到识别代码
        // 识别陀螺函数,输入当前帧是否识别now_status(0,1),上一帧是否识别pre_status(0,1),
        // 此外,还需要输入距离和yaw轴角度(没有识别到随便给就好)
        asc.judge_is_spinning(armordetection._flag, pre_flag, armordetection.distance, armordetection.yaw_angle);
        // 如果当前帧识别到了,那么往队列插入新的值,
        // 注意:算法逻辑上,必须先识别陀螺,再往队列里面插入值保存信息
        if(armordetection._flag)
        {
           asc.insert_values(armordetection.pitch_angle, armordetection.yaw_angle, armordetection.distance);
        }




///////////////////作为下一帧的上一帧///////////////
       pre_flag = armordetection._flag;
//       pre_yaw_angle = vdata.yaw_angle.f;


        // 展示检测效果
//        armordetection.put_text_into_img(vdata);
//        armordetection.show_img();
//        waitKey(1);



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


}




#endif
