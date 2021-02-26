#include "numpredict.h"

#define SHOW_NUM
//#define VOTE
Numpredict::Numpredict()
{

    memset(possibleNum, 0, sizeof(possibleNum));
    frames = 0;
    gamma=0.4;

    string modelTxt = "/home/nuc/Desktop/infantry_armor_detect/lenet_emhance/lenet_deploy_9.prototxt";
    string modelBin = "/home/nuc/Desktop/infantry_armor_detect/lenet_emhance/lenet.caffemodel";


    try {
        net = dnn::readNetFromCaffe(modelTxt, modelBin);
    }
    catch (cv::Exception& ee)
    {
        cerr << "Exception: " << ee.what() << endl;
        if (net.empty())
        {
            cout << "cannot load the network by using the flowing filed:" << endl;
            cout << "'modelTxt:" << modelTxt << endl;
            cout << "modelBin: " << modelBin << endl;
            exit(-1);
        }
    }

}

void Numpredict::getMaxClass(const Mat& probBlob, int* classId, double* classProb)
{
    Mat probMat = probBlob.reshape(1, 1);//单通道，行数为1，列数由自动求出。
    Point classNumber;
    //在数组中找到全局的最小值和最大值以及它们的位置
    //（prob，最小值，最大值，最小值位置，最大值位置）
    minMaxLoc(probMat, NULL, classProb, NULL, &classNumber);
    *classId = classNumber.x;
}

void Numpredict::gamma_trans(Mat &img_num,Mat& img_gamma)
{
    double temp;
    uchar lutData[256];
    double over_255=1.0/255.0;
    for(int i=0;i<256;i++)
    {
        temp=double(i)*over_255;
        temp=pow(temp,gamma);
        lutData[i]=uchar(temp*255);
    }
    Mat lut(1,256,CV_8UC1,lutData);
    cv::LUT(img_num,lut,img_gamma);
}

int Numpredict::num_predict(Mat &img)
{
    Mat img_float,img_gamma;
    gamma_trans(img,img_gamma);
#ifdef SHOW_NUM
    namedWindow("gamma_num",WINDOW_NORMAL);
    imshow("gamma_num",img_gamma);
#endif
    auto_exposure(0.4,img_gamma,0);


    resize(img_gamma, img_gamma, Size(28, 28));

    //cvtColor(img, img, COLOR_BGR2GRAY);
//    img.convertTo(img_float, CV_32FC3, 0.00390625);
    img_gamma.convertTo(img_float, CV_32FC3);

    Mat inputBlob = dnn::blobFromImage(img_float);
    Mat pred;
    net.setInput(inputBlob, "data");

    pred = net.forward("prob");
    //cout << pred << endl;
    int classId;
    double classProb;
    getMaxClass(pred, &classId, &classProb);

    cout << "/*****************/" << endl;
//    cout<<" predict result: " << pred.at<float>(0, classId) ;
    cout<<" predict result: " << pred<<endl ;
    if(pred.at<float>(0, classId) < 0.80)
    {
        classId=8;
    }
    cout<< "   predict class : " << classId << endl;


    /*cout << "Best class: " << classId << endl;
    cout << "Probability: " << classProb * 100 << "%" << endl;*/

    return classId;
}

//enemy_color: 0 is blue,1 is red
float Numpredict::auto_exposure(float cur_exposure_time,Mat &img_gamma,int enemy_color)
{
    int dist_thresh=120*img_gamma.rows*img_gamma.cols;
    vector<Mat> RGBchannels;
    split(img_gamma,RGBchannels);
    Mat img_gray;
    cvtColor(img_gamma,img_gray,COLOR_BGR2GRAY);
    int DistSum=0;

    for(int i=0;i<img_gamma.rows;i++)
    {
        uchar* data_gray=img_gray.ptr<uchar>(i);
        uchar* data_RGB;
        if(enemy_color==0)  //blue
            data_RGB=RGBchannels[0].ptr<uchar>(i);
        if(enemy_color==1)  //red
            data_RGB=RGBchannels[2].ptr<uchar>(i);
        for(int j=0;j<img_gamma.cols;j++)
        {
            int Dist=(int)(*data_RGB-*data_gray);
            if(Dist>2)
            {
                DistSum+=Dist;
            }
        }
    }
    gamma=cur_exposure_time+(DistSum-dist_thresh)*1e-7;
    cout<< "dist_thresh "<<dist_thresh<<endl;
    cout<<"DistSum-dist_thresh "<<DistSum-dist_thresh*1e-7<<endl;
    cout<<"gamma "<<gamma<<endl;
    return cur_exposure_time+(DistSum-dist_thresh)*1e-7;
}

int Numpredict::predict_project(Mat& img_num, bool is_the_same_object)
{
    ++frames;
    cout<<"frames: "<<frames <<endl;
#ifdef VOTE
    if (is_the_same_object == false)
    {
        memset(possibleNum, 0, sizeof(possibleNum));
        frames = 0;
        return 8;
    }
    if (frames > 30)
    {
        return fixed_label;
    }
#endif
    int label = num_predict(img_num);
    ++possibleNum[label];

    int possible_number = 0;
    for (int i = 0; i < 9; i++)
    {
        if (possibleNum[i] > possibleNum[possible_number])
        {
            possible_number = i;
        }
    }
    fixed_label = possible_number;
    return fixed_label;
}
