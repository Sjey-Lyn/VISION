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
#include<antispinning.h>
#include"deal_drop_frame.h"
#include"save_video.h"

//#include "object_predict.h"
//#include "rp_kalman.h"

using namespace cv;
using namespace std;

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
volatile unsigned int save_image_index;


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
int mode = 1, shoot_speed = 0, my_color = 1;
///////////////////补充函数///////////////////
void Taitou(double &offset_pit, double last_dist);
void init_camera();
///////////////////////////////////////////




void ImageConsProd::ImageProducer()
{

    init_camera();
    while(1){
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS){
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);

            cv::Mat src(
                    cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),
                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer);
            CameraReleaseImageBuffer(hCamera,pbyBuffer);
//            time_now = (double)getTickCount();
//            cout<<"produce time"<<(time_now - time_last)/getTickFrequency()*1000<<"ms"<<endl;
//            time_last = time_now;
            while (prdIdx - csmIdx >= BUFFER_SIZE);  // 不能注释，注释了会出现零图，然后导致掉帧发生
            data[prdIdx % BUFFER_SIZE].img = src;
            prdIdx++;
//            cout<<prdIdx<<endl;
        }

    }

    // 反初始化再free
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
}



void ImageConsProd::ImageConsumer()
{
///////////串口///////////
    port.initSerialPort();

///////////能量机关//////////
    Energy energy;
    energy.isRed(false);
    ArmorRect present, former;
////////////反陀螺///////////
    anti_spinning_controller asc;
/////////////掉帧//////////
    drop_frame_ctrl dfc;
    int pre_flag=0;
    double pre_yaw_angle = 0;
///////////数字识别//////////
    Numpredict num_pred;

///////////自瞄/////////////
    Mat frame;
    queue<Rect> last_roi;   //存放前几帧的roi
    last_roi.push(Rect(0,0, frame_W, frame_H));  // 第一帧roi是整张图片
    double roi_scale=1.5;   // roi相对于旋转矩形的放大比例
    int loss_frame = 0;      // 掉帧系数

#ifdef DEAL_DIAOZHEN
    ofstream send_yaw("send_yaw.txt");
    ofstream send_pitch("send_pitch.txt");
    ofstream yaw("yaw.txt");
    ofstream pitch("pitch.txt");
#endif

    // 相机内参
    Mat cam = (Mat_<double>(3, 3) << 2000, 0, 700,
                                     0.0000000000000000, 2000, 265.3870,
                                     0.0000000000000000, 0.0000000000000000, 1.0000000000000000);
    // 畸变系数
    Mat dis = (Mat_<double>(5, 1) << 0.0576, 0.4540, -0.0014,
                                    -0.0118, 0.0000000000000000);


    //double startTime, endTime;
    while (1){
        //startTime = getTickCount();
//        cout<<endl;
        while(prdIdx - csmIdx == 0);                                         //线程锁
        port.get_Mode(mode, shoot_speed, my_color);
        data[csmIdx % BUFFER_SIZE].img.copyTo(frame);                         //将Producer线程生产的图赋值给src
        ++csmIdx;                                                           //解锁，生产下一张图
//        double t1 = getTickCount();
//        cout<<"csmidx"<<csmIdx<<endl;
        if(frame.empty()){
            cout<<"frame is empty"<<endl;
            continue;
        }
        time_now = (double)getTickCount();
//        cout<<"delta time"<<(time_now - time_last) / getTickFrequency()*1000<< "ms"<<endl;
        time_last = time_now;

//        cout<<"当前mode"<<mode<<endl;
        if(mode== 1)    //自瞄
        {
          Armordetection armordetection = Armordetection(frame, last_roi.back(), loss_frame);
          armordetection.begin_to_detect(cam, dis,        //Mat c, Mat d ------>相机内参, 畸变系数
                                       roi_scale,         //double roi_scale -----> roi相对于旋转矩形的放大比例
                                       armor_h, armor_w,  //double armorH, double armorW -----> 检测的装甲板的实际高，宽
                                       1,                 //bool use_roi -----> 是否使用roi，建议调试时不使用，以观察算法的准确性，比赛时使用，以提高检测速度
                                       my_color,          //int my_color 获取我方颜色
                                       0,                 //bool show_mask ------> 是否展示图像预处理效果
                                       0, 0 ,             //bool show_light_filter_result, bool show_match_result ----> 是否展示筛选灯条，匹配灯条的效果
                                       0,                 // bool get_num_image -----> 是否提取装甲板上的数字,1
                                       100, 0,            //int num_img_size, bool show_num_img ---> 提取装甲板上的数字的图像大小， 以及是否展示提取的数字
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

        dfc.insert_values(armordetection._flag,armordetection.pitch_angle, armordetection.yaw_angle, armordetection.distance);
        dfc.get_aspeed();

        // 嵌入到识别代码
        // 识别陀螺函数,输入当前帧是否识别now_status(0,1),上一帧是否识别pre_status(0,1),
        // 此外,还需要输入距离和yaw轴角度(没有识别到随便给就好)
//        asc.judge_is_spinning(armordetection._flag, pre_flag, armordetection.distance, armordetection.yaw_angle);
        // 如果当前帧识别到了,那么往队列插入新的值,
        // 注意:算法逻辑上,必须先识别陀螺,再往队列里面插入值保存信息
//        if(armordetection._flag)
//        {
//           asc.insert_values(armordetection.pitch_angle, armordetection.yaw_angle, armordetection.distance);
//        }
        // 如果识别出陀螺状态,采用陀螺处理,
        // 并将要发给电控的三个值置为asc类的spinning_pitch、spinning_yaw、spinning_distance
        int flag_temp = 0;
        if(asc.is_spinning && flag_temp)
        {
           cout<<"进入了反陀螺"<<endl;
           //根据存储的信息计算定住的位置
           asc.spinning_process(0);
           vdata.pitch_angle.f = asc.spinning_pitch;
           cout<<"error:"<<vdata.pitch_angle.f<<endl;
           vdata.yaw_angle.f = asc.spinning_yaw;
           vdata.dis.f = asc.spinning_distance;
           vdata.isFindTarget = armordetection._flag;
           vdata.drop_frame=0;
           vdata.anti_top=1;
           vdata.anti_top_change_armor = 0;
           vdata.nearFace = 0;
           vdata.isfindDafu= 0;

        }
        else
        {
        // 没有识别出陀螺状态时,需要把asc的spinning_process_param置0
        // 发送电控的数据按正常情况下发送即可
           asc.spinning_process_param = 0;
           if(dfc.drop_frame)
           {
//               cout<<"掉帧了"<<endl;
               vdata.dis.f = dfc.drop_distance;
               vdata.pitch_angle.f = dfc.drop_pitch;
               vdata.yaw_angle.f = dfc.drop_yaw;
               vdata.isFindTarget = 1;
               if(fabs(pre_yaw_angle - dfc.drop_yaw) > 4)
               {
                  vdata.anti_top_change_armor=1;
//                  cout<<"-----------------切换了-------------------"<<endl;
               }
               else
               {
                  vdata.anti_top_change_armor=0;
               }
               vdata.anti_top=0;
               vdata.nearFace = 0;
               vdata.isfindDafu= 0;
           }
           else
           {
               vdata.dis.f = armordetection.distance;
               vdata.pitch_angle.f = armordetection.pitch_angle;
               vdata.yaw_angle.f = armordetection.yaw_angle;
               vdata.isFindTarget = armordetection._flag;
               if(fabs(pre_yaw_angle - armordetection.yaw_angle) > 4)
               {
                  vdata.anti_top_change_armor=1;
//                  cout<<"-----------------切换了-------------------"<<endl;
               }
               else
               {
                  vdata.anti_top_change_armor=0;
               }
               vdata.drop_frame=0;
               vdata.anti_top=0;
               vdata.nearFace = 0;
               vdata.isfindDafu= 0;
           }

           double pit_offset = 0;
           if(armordetection._flag || dfc.drop_frame)
           {
               Taitou(pit_offset, armordetection.distance);
               cout<<"补偿值"<<pit_offset<<endl;
               cout<<"未补偿前"<< vdata.pitch_angle.f<<endl;
               vdata.pitch_angle.f += pit_offset;
               cout<<"补偿后"<<vdata.pitch_angle.f<<endl;
           }

#ifdef DEAL_DIAOZHEN

       send_yaw <<endl<<vdata.yaw_angle.f;
       send_pitch << endl << vdata.pitch_angle.f;
       yaw <<endl<< armordetection.yaw_angle;
       pitch << endl << armordetection.pitch_angle;
#endif
        }

///////////////////作为下一帧的上一帧///////////////
       pre_flag = armordetection._flag;
       pre_yaw_angle = vdata.yaw_angle.f;

////////////////////抬头补偿////////////////////
//        double pit_offset = 0;
//        if(armordetection._flag || dfc.drop_frame)
//        {
//            Taitou(pit_offset, armordetection.distance);
////            cout<<"补偿值"<<pit_offset<<endl;
////            cout<<"未补偿前"<< vdata.pitch_angle.f<<endl;
//            vdata.pitch_angle.f += pit_offset;
////            cout<<"补偿后"<<vdata.pitch_angle.f<<endl;
//        }
///////////////////////////////////////////////
//        cout<<"抬头量"<<vdata.pitch_angle.f<<"距离"<<armordetection.distance<<endl;

        // 展示检测效果
//        armordetection.put_text_into_img(vdata);
//        armordetection.show_img();
//        waitKey(1);
          }


           if( mode == 2)
           {               
               double BUFF_Exposure = 2000;
               iStatus = CameraSetExposureTime(hCamera, BUFF_Exposure);
//               tSdkImageResolution pImageResolution = {0};
//               pImageResolution.iIndex = 0xff; //表示自定义分辨率（roi）
//               pImageResolution.iHeight = 1024;
//               pImageResolution.iHeightFOV = 1024;
//               pImageResolution.iVOffsetFOV = 0;
//               pImageResolution.iHOffsetFOV = 160;
//               pImageResolution.iWidth = 960;
//               pImageResolution.iWidthFOV = 960;
//               iStatus = CameraSetImageResolution(hCamera, &pImageResolution);
               int THRESH_BR = 57;//56
               int THRESH_GRAY = 128;//128
               Mat mask;
               energy.is_big = 0 ;
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

//        double t2 = getTickCount();
//        cout<<"comsumer time"<<(t2 - t1)/getTickFrequency()*1000<<endl;
//        cout<<"--------------------------"<<endl;
        port.TransformData(vdata);
        port.send();

//        endTime = getTickCount();
//        std::cout<<(endTime - startTime)/getTickFrequency()<<std::endl;



    }

#ifdef DEAL_DIAOZHEN
    yaw.close();
    pitch.close();
    send_pitch.close();
    send_yaw.close();
#endif
}



void ImageConsProd::ImageCapture()
{

    cout << " ------ IMAGE WRITE TASK ON !!! ------" << endl;
    SaveVideo writer;
//    INFO(writer.getState());
    while(writer.getState()){
        while(static_cast<int>(prdIdx - save_image_index) <= 0){

        };
        double t1 = getTickCount();
        Mat img_tmp;
        data[csmIdx % BUFFER_SIZE].img.copyTo(img_tmp);
//        imshow("img",img_tmp);
//        waitKey(1);
        writer.updateImage(img_tmp);
        double t2 = getTickCount();
//        cout<<"capture time"<<(t2 - t1)/getTickFrequency()*1000<<"ms"<<endl;
        save_image_index++;
    }
}





void Taitou(double &offset_pit,double last_dist)
{

    if(last_dist >= 250 && last_dist < 750)
    {
         offset_pit = 4;
//        //低速射击
//        if(shoot_speed == 15)
//        {
//            offset_pit = 4;
////            cout << "短距离低速抬头补偿" << endl;
//        }
//        //中速射击
//        if(shoot_speed == 18)
//        {
//            offset_pit = 0;
////            cout << "短距离中速抬头补偿" << endl;
//        }
//        //高速射击
//        if(shoot_speed == 30)
//        {
//            offset_pit = 0;
////            cout << "短距离高速抬头补偿" << endl;
//        }
    }

    else if(last_dist >= 750 && last_dist < 1250)
        offset_pit = 3.8;
    else if(last_dist >= 1250 && last_dist < 1750)
        offset_pit = 3.6;
    else if(last_dist >= 1750 && last_dist < 2250)
        offset_pit = 3.4;
    else if(last_dist >= 2250 && last_dist < 3250)
        offset_pit = 3.3;
    else if(last_dist >= 3250 && last_dist < 3750)
        offset_pit = 3.5;
    else if(last_dist >= 3750 && last_dist < 4250)
        offset_pit =4;

}


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
//    CameraSetExtTrigSignalType(hCamera,EXT_TRIG_LEADING_EDGE) ;
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
    iStatus =CameraSetGamma(hCamera,100);
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
    pImageResolution.iVOffsetFOV = 304;
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




#endif //tiaoche
