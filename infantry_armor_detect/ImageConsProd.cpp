#define tiaoche
#ifdef tiaoche

#include<iostream>
#include<opencv2/opencv.hpp>
#include"GxIAPI.h"
#include "DxImageProc.h"
#include "numpredict.h"
#include<ImageConsProd.h>
#include<armordetection.h>
#include<rp_kalman.h>
#include<queue>
#include<serialport.h>
#include<CRC_Check.h>
#include<Energy.h>
#include<CameraApi.h>
#include<CameraDefine.h>
#include<CameraStatus.h>
#include<fstream>
//#include "object_predict.h"
//#include "rp_kalman.h"

using namespace cv;
using namespace std;


///////////////////掉帧处理全局变量///////////////////
//#define diaozhen
float last_pitch=0,last_yaw=0,last_dis=0;     //存掉帧信息（pitch，yaw，dist）的全局变量
int find_index=0;                            //持续帧数
int lost_index=0;                             //掉帧
///////////////////掉帧处理全局变量2///////////////////
//#define DEAL_DIAOZHEN
#define VECTOR_MAX 20
#define LOST_OBJECT 10
int lost_time = 0;
vector<double>yaw_vec;
vector<double>pitch_vec;
double last_dantance;
/////////////////////////////////////////////////////


///////////////////相机的东西///////////////////
unsigned char * g_pRgbBuffer;     //处理后数据缓存区
int hCamera;
int                     iStatus=-1;
tSdkFrameHead           sFrameInfo;
BYTE*			        pbyBuffer;
//////////////////////////////////////////////

double time_now;    //当前时间
double time_last;   //上一次时间

volatile unsigned int prdIdx;   //线程锁
volatile unsigned int csmIdx;   //线程锁


#define BUFFER_SIZE 1   //图片缓冲区
struct ImageData {
    Mat img;
    unsigned int frame;
};
ImageData data[BUFFER_SIZE];


//VideoWriter vw("videoTest.avi",VideoWriter::fourcc('M','J','P','G'),25,Size(1280,720),true); // 定义写入视频对象


///////////////////串口///////////////////
SerialPort port("/dev/ttyUSB0");    //ls /dev/
VisionData vdata;
int mode = 1  , buff_four, shoot_speed, my_color;
/////////////////////////////////////////






void init_camera()
{
    ///////////////////初始化///////////////////
    //////////////// Describe Appliance Info ////////////////////
    int                     iCameraCounts = 1;
    tSdkCameraDevInfo       tCameraEnumList;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    int                     channel=3;
    ////////////////// Init Camera //////////////////////////////
    CameraSdkInit(1);
    ////////////////// Enum Devices /////////////////////////////
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);   //成功时返回0
    printf("Find camera count = %d\n", iCameraCounts);      //连接的设备个数
    //没有连接设备
    if(iCameraCounts==0){
        return  ;
    }
    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
    if (iStatus != CAMERA_STATUS_SUCCESS){
        return ;
    }else{
        cout << "Init Camera Succeeeded!" << endl;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(
                tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
    CameraSetAeState(hCamera, FALSE);   //FALSE为手动曝光，TRUE为自动曝光
    //////////////////////////////////////////////////////////////




    ///////////////////设置硬触发///////////////////
    //    CameraSetTriggerMode(hCamera,2);
    //    // 设置硬触发的触发方式
    //    CameraSetExtTrigSignalType(hCamera,EXT_TRIG_LOW_LEVEL) ;
    //    //    EXT_TRIG_LEADING_EDGE = 0,     //上升沿触发，默认为该方式
    //    //    EXT_TRIG_TRAILING_EDGE,        //下降沿触发
    //    //    EXT_TRIG_HIGH_LEVEL,           //高电平触发
    //    //    EXT_TRIG_LOW_LEVEL             //低电平触发


    //////////////////////set param ///////////////////////////////////
    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    //调曝光时间
    // iStatus = CameraSetExposureTime(hCamera, EXPOSURE);
    if (iStatus != CAMERA_STATUS_SUCCESS)
        return ;
        else cout << "Init SDK Succeeeded!" << endl;

    //读取曝光时间

    double Exposure = 2000;
    iStatus = CameraSetExposureTime(hCamera, Exposure);
    double baoguang=0;
    iStatus =CameraGetExposureTime(hCamera, &baoguang);
    cout<<"曝光时间为:  "<<baoguang<<endl;

    //读gama
    int gama=0;
    // iStatus =CameraSetGamma(hCamera,2);
    iStatus =CameraGetGamma(hCamera,&gama);
    cout<<"伽马值为：   "<<gama<<endl;

    //读对比度
    int contrast=0;
    // iStatus =CameraSetGamma(hCamera,2);
    iStatus =CameraGetGamma(hCamera,&contrast);
    cout<<"对比度为：   "<<contrast<<endl;

    // saturation

    //读增益
    int r=0,g=0,b=0;
    // iStatus =CameraSetGain(hCamera,r,g,b);
    iStatus =CameraGetGain(hCamera,&r,&g,&b);
    cout<<"增益值为：  r "<<r<<"   g:   "<<g<<"b  :"<<b<<endl;


    //CameraGetGain
    //////////////////// 设置分辨率 //////////////////////

    tSdkImageResolution pImageResolution = {0};
    pImageResolution.iIndex = 0xff; //表示自定义分辨率（roi）
    pImageResolution.iHeight = 720;
    pImageResolution.iHeightFOV = 720;
    pImageResolution.iVOffsetFOV = 0;
    pImageResolution.iHOffsetFOV = 0;
    pImageResolution.iWidth = 1280;
    pImageResolution.iWidthFOV = 1280;
    iStatus = CameraSetImageResolution(hCamera, &pImageResolution);

    /*
        设置图像处理的输出格式，彩色黑白都支持RGB24位
    */
    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }



}




void ImageConsProd::ImageProducer()
{

    init_camera();
    while(1){
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS){
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);

            // 得到一张图片
            cv::Mat src(
                    cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),
                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer);
            CameraReleaseImageBuffer(hCamera,pbyBuffer);
            while (prdIdx - csmIdx >= BUFFER_SIZE)  // 不能注释，注释了会出现零图，然后导致掉帧发生

            data[prdIdx % BUFFER_SIZE].img = src;
            prdIdx++;

        }

    }

    // 反初始化再free
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
}



void ImageConsProd::ImageConsumer()
{
    port.initSerialPort();
    Energy energy;
    ArmorRect present, former;
    Numpredict num_pred;
    energy.isRed(false);
    Mat frame;
    queue<Rect> last_roi;   //存放前几帧的roi
    last_roi.push(Rect(0,0, frame_W, frame_H));  // 第一帧roi是整张图片
    double roi_scale=0.4;   // roi相对于旋转矩形的放大比例
    int loss_frame = 0;      // 掉帧系数

    ofstream send_yaw("send_yaw.txt");
    ofstream send_pitch("send_pitch.txt");
    ofstream yaw("yaw.txt");
    ofstream pitch("pitch.txt");

    // 相机内参
    Mat cam = (Mat_<double>(3, 3) << 2406.8, 0, 560.7913,
                                     0.0000000000000000, 2415, 736.8329,
                                     0.0000000000000000, 0.0000000000000000, 1.0000000000000000);
    // 畸变系数
    Mat dis = (Mat_<double>(5, 1) <<-0.0292, 0.1877, 0.0168,
                                    0.0168, 0.0000000000000000);

///////////////////數字識別 拍圖片用///////////////////
//    string imgname1;
//    int f = 0;
//    string imgname2;
//    int fff = 1000;
////////////////////////////////////////////////////
    //double startTime, endTime;
    while (1){
        //startTime = getTickCount();
        while(prdIdx - csmIdx == 0);                                         //线程锁
        port.get_Mode(mode, shoot_speed, my_color);


        data[csmIdx % BUFFER_SIZE].img.copyTo(frame);                         //将Producer线程生产的图赋值给src
        ++csmIdx;                                                           //解锁，生产下一张图
        double t1 = getTickCount();
//        cout<<"csmidx"<<csmIdx<<endl;
        if(frame.empty()){
            continue;
        }
        time_now = (double)getTickCount();
//        cout<<"delta time"<<(time_now - time_last) / getTickFrequency()*1000<< "ms"<<endl;
        time_last = time_now;


        if(mode==1 )    //自瞄
        {
          Armordetection armordetection = Armordetection(frame, last_roi.back(), loss_frame);
          armordetection.begin_to_detect(cam, dis,        //Mat c, Mat d ------>相机内参, 畸变系数
                                       roi_scale,       //double roi_scale -----> roi相对于旋转矩形的放大比例
                                       armor_h, armor_w,//double armorH, double armorW -----> 检测的装甲板的实际高，宽
                                       0,               //bool use_roi -----> 是否使用roi，建议调试时不使用，以观察算法的准确性，比赛时使用，以提高检测速度
                                       1,               //int color_detection_mode 图像预处理提取二值图的模式
                                       1,               //bool show_mask ------> 是否展示图像预处理效果
                                       1, 1 ,           //bool show_light_filter_result, bool show_match_result ----> 是否展示筛选灯条，匹配灯条的效果
                                       0,               // bool get_num_image -----> 是否提取装甲板上的数字,
                                       100, 0,          //int num_img_size, bool show_num_img ---> 提取装甲板上的数字的图像大小， 以及是否展示提取的数字
                                       num_pred);
        //这里记录前10帧的roi信息
        if ( last_roi.size() >= 10)
        {
            last_roi.pop();
        }
        last_roi.push(armordetection._next_roi);

        // 计算掉帧数
        if(armordetection._flag == 1)
        {
            loss_frame = 0;
        }
        else
        {
            loss_frame++;
        }



#ifdef DEAL_DIAOZHEN
        if(armordetection._flag )
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


            if(pitch_vec.size() < VECTOR_MAX)
            {
                pitch_vec.push_back(armordetection.pitch_angle);
            }
            else
            {
                vector<double>::iterator first = pitch_vec.begin();
                pitch_vec.erase(first);
                pitch_vec.push_back( armordetection.pitch_angle);
            }


        }
        else
        {
            lost_time++;
        }



        if( lost_time > 0 && lost_time < LOST_OBJECT)
        {
            double yaw_sum1, yaw_sum2;
            double pit_sum1, pit_sum2;
            vdata.dis.f = last_dantance;
            vdata.isFindTarget = 1 ;
            vdata.buff_change_four=0;
            vdata.anti_top=0;
            vdata.anti_top_change_armor = 0;
            vdata.nearFace = 0;
            vdata.isfindDafu= 0;


            if(yaw_vec.size()%2 == 1 && yaw_vec.size() > 0)
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

                double send_yaw_angle = yaw_vec.back() + (yaw_sum2 - yaw_sum1)/ (yaw_vec.size() / 2 )/(yaw_vec.size() / 2) * lost_time ;
                vdata.yaw_angle.f = send_yaw_angle;

            }

            else if(yaw_vec.size() > 0)
            {
                for(int i = 0 ; i < yaw_vec.size() / 2; i++)
                {
                    yaw_sum1 += yaw_vec.at(i);
                }

                for(int i = yaw_vec.size() / 2 ; i < yaw_vec.size() ; i++)
                {
                    yaw_sum2 += yaw_vec.at(i);
                }

                double send_yaw_angle = yaw_vec.back() + (yaw_sum2 - yaw_sum1)/ (yaw_vec.size() / 2) /(yaw_vec.size() / 2) * lost_time ;
                vdata.yaw_angle.f = send_yaw_angle;
            }



            if(pitch_vec.size()%2 == 1 && pitch_vec.size() > 0)
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

                double send_pitch_angle = pitch_vec.back() + (pit_sum2 - pit_sum1)/ (pitch_vec.size() / 2) /(pitch_vec.size() / 2) * lost_time ;
                vdata.pitch_angle.f = send_pitch_angle;

           }
            else if(pitch_vec.size() > 0)
            {
                for(int i = 0 ; i < pitch_vec.size() / 2; i++)
                {
                    pit_sum1 += pitch_vec.at(i);
                }

                for(int i = pitch_vec.size() / 2 ; i < pitch_vec.size() ; i++)
                {
                    pit_sum2 += pitch_vec.at(i);
                }

                double send_pitch_angle = pitch_vec.back() + (pit_sum2 - pit_sum1)/ (pitch_vec.size() / 2) /(pitch_vec.size() / 2) *lost_time ;
                vdata.pitch_angle.f = send_pitch_angle;
            }

        }
        else
        {
            vdata.dis.f = armordetection.distance;
            vdata.pitch_angle.f = armordetection.pitch_angle;
            vdata.yaw_angle.f = armordetection.yaw_angle;
            vdata.isFindTarget = armordetection._flag;
            vdata.buff_change_four=0;
            vdata.anti_top=0;
            vdata.anti_top_change_armor = 0;
            vdata.nearFace = 0;
            vdata.isfindDafu= 0;
        }

       send_yaw <<endl<<vdata.yaw_angle.f;
       send_pitch << endl << vdata.pitch_angle.f;


#endif




#ifdef diaozhen
///////////////////////////////////////// 掉帧处理 START////////////////////////////////
        if(armordetection._flag)
        {
             find_index++;        //持续识别帧数
             lost_index = 0;
        }


        if(!armordetection._flag)
        {
            find_index = 0;        //如果有一帧识别不到装甲板就清零
            lost_index++;
        }

#ifdef DROP_PROCESS
    cout << "DROP_PROCESS——持续帧数为：   " <<find_index++ << " 帧" << endl;
    cout << "DROP_PROCESS——掉帧帧数为：  " << lost_index<< " 帧" << endl;
    cout<<"--------------------------------------------------"<<endl;
#endif //DROP_PROCESS
    if(find_index >= 2){           //当持续识别大于两帧则将当前信息存起来
        last_yaw = armordetection.yaw_angle;  //将当前信息保存起来，为之后可能存在的掉帧的情况做准备
        last_pitch = armordetection.pitch_angle;
        last_dis = armordetection.distance;
    }
    if(lost_index > 0 && lost_index < 8 && fabs(last_pitch) < 8 ){    //如果掉帧帧数在0-8帧内且上一次pitch角不太大
        armordetection._flag = 1;                                      //默认为掉帧处理
        //DROP_PROCESS 调参说明：下面last_x后面的参数表示减小预测值
        armordetection.yaw_angle = last_yaw * 0.75;                             //角度减小（可以适当调）
        armordetection.pitch_angle = last_pitch;                                    //俯仰角不变
        armordetection.distance = last_dis;                                //距离赋值
    }
#endif





        vdata.dis.f = armordetection.distance;
        vdata.pitch_angle.f = armordetection.pitch_angle;
//        cout<<"pitch="<<armordetection.pitch_angle*180/3.14<<endl;
        vdata.yaw_angle.f = armordetection.yaw_angle;
        vdata.isFindTarget = armordetection._flag;
        vdata.buff_change_four=0;
        vdata.anti_top=0;
        vdata.anti_top_change_armor = 0;
        vdata.nearFace = 0;
        vdata.isfindDafu= 0;

        yaw <<endl<< armordetection.yaw_angle;
        pitch << endl << armordetection.pitch_angle;

        // 展示检测效果
        armordetection.put_text_into_img(vdata);
        armordetection.show_img();
        waitKey(1);



///////////////////数字识别拍图片用///////////////////
//            char key = waitKey(1);
//            if(key =='q')
//            {
//                imgname1 = to_string(f++)+".jpg";
//                imwrite(imgname1,frame);
//                imgname2 = to_string(fff++)+".jpg";
//                imwrite(imgname2,armordetection.img_gramm);
//            }
////////////////////////////////////////////////////



          }


           if( mode == 2)
           {
//               double val = 2000;
//               cout << "val = " << val << endl;
//               GX_STATUS status = GX_STATUS_SUCCESS;
//               status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, val);
               int THRESH_BR = 57;//56
               int THRESH_GRAY = 128;//128
               Mat mask;
               energy.is_big = 1 ;
               energy.getTime = 22;// small:20, big:13 // newsmall: 22(5/5, 100%)
               energy.setThresh(THRESH_BR, THRESH_GRAY);
               energy.videoProcess(frame, mask, present, former);
               energy.pre_x = present.center.x;
               energy.pre_y = present.center.y;
               //std::cout<<energy.pre_x<<" "<<energy.pre_y<<std::endl;
               energy.updateSpeed(present, former, frame);
               //energy.predict(present, frame);
//               vdata.pitch_angle.f = 1000;
//               vdata.yaw_angle.f = 1000;
               vdata.pitch_angle.f = energy.pre_y;
               vdata.yaw_angle.f = energy.pre_x;
               vdata.isfindDafu =energy.is_find;
               //vdata.anti_top_change_armor = energy.time2fire;
               vdata.anti_top_change_armor = 1;

               namedWindow("Dafu");
               namedWindow("DafuBinary");
               imshow("Dafu", frame);
               imshow("DafuBinary", mask);
               waitKey(1);
           }

        double t2 = getTickCount();
//        cout<<"comsumer time"<<(t2 - t1)/getTickFrequency()*1000<<endl;
//        cout<<"--------------------------"<<endl;
        port.TransformData(vdata);
        port.send();

//        endTime = getTickCount();
//        std::cout<<(endTime - startTime)/getTickFrequency()<<std::endl;



    }


    yaw.close();
    pitch.close();
    send_pitch.close();
    send_yaw.close();
}

#endif //tiaoche

