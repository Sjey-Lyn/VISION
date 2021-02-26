#include<armordetection.h>


Light::Light(vector<Point>& contour)
{
    _contour = contour;
}

//获取灯条面积
int Light::get_area()
{
    return contourArea(_contour);
}

//获取旋转矩形
RotatedRect Light::get_rotatderect()
{
    return minAreaRect(_contour);
}

//获取灯条的中心点(注意是在roi下的坐标，要想获得原图坐标，还要加上roi的top—left点)
Point Light::get_center()
{
    Moments mu;
    mu = moments(_contour);
    return Point(mu.m10/mu.m00, mu.m01/mu.m00);
}

//获取灯条最左点(注意是在roi下的坐标，要想获得原图坐标，还要加上roi的top—left点)
Point Light::Left_point()
{
    Point Left = *min_element(this->_contour.begin(), this->_contour.end(),
      [](const Point& lhs, const Point& rhs) {return lhs.x < rhs.x;});
    return Left;
}

//获取灯条最右点(注意是在roi下的坐标，要想获得原图坐标，还要加上roi的top—left点)
Point Light::Right_point()
{
    Point Right = *max_element(this->_contour.begin(), this->_contour.end(),
      [](const Point& lhs, const Point& rhs) {return lhs.x < rhs.x;});
    return Right;
}

//获取灯条最上点(注意是在roi下的坐标，要想获得原图坐标，还要加上roi的top—left点)
Point Light::Top_point()
{
    Point Top = *min_element(this->_contour.begin(), this->_contour.end(),
      [](const Point& lhs, const Point& rhs) {return lhs.y < rhs.y;});
    return Top;
}

//获取灯条最下点(注意是在roi下的坐标，要想获得原图坐标，还要加上roi的top—left点)
Point Light::Bottom_point()
{    Point Bottom = *max_element(this->_contour.begin(), this->_contour.end(),
      [](const Point& lhs, const Point& rhs) {return lhs.y < rhs.y;});
     return Bottom;
}

// 将灯条拟合成直线，并返回该直线的斜率
double Light::line_k()
{
    Vec4f line_para;
    fitLine(_contour, line_para, DIST_L2, 0, 1e-2, 1e-2);
    double k = line_para[1] / line_para[0];
    return k;
}





Armordetection::Armordetection(Mat img, Rect roi, int loss_frame_k)
{
    _img = img.clone();
    _frame = img;
    _now_roi = roi;
    _lossFrameK = loss_frame_k;
    distance = 0 ;
    pitch_angle = 0 ;
    yaw_angle = 0 ;
}

void Armordetection::set_img(bool use_roi)
{

    if(use_roi){
        _roi = Mat(_img, _now_roi);
     }
     else
     {
        _roi = _img.clone();
        _now_roi = Rect(0, 0, _img.cols, _img.rows);
        _lossFrameK = 0;
     }
}

void Armordetection::color_detection(int color_detection_mode, bool show_mask)  // 后期再加入其他方法
{
    int enermy_color = 1;              //1 :蓝色  0 :红色
    if(color_detection_mode == 1){      //红蓝通道相减，漫水填充补空洞
       if(enermy_color == 1){
         Mat thres_whole;
         vector<Mat> splited;
         Mat color,mask;
         int thres_max_color_blue = 30;
         split(_roi,splited);
         cvtColor(_roi,thres_whole,CV_BGR2GRAY);

         threshold(thres_whole,thres_whole,60,255,THRESH_BINARY);
//         namedWindow("thresh_whole", 0);
//         imshow("thresh_whole", thres_whole);
         subtract(splited[0],splited[2], color);
         threshold(color, mask, thres_max_color_blue,255,THRESH_BINARY);
         dilate(mask, mask,getStructuringElement(MORPH_RECT,Size(3,3)));
//         namedWindow("th", 0);
//         imshow("th", mask);
         Size m_Size = mask.size();
         Mat Temp = Mat::zeros(m_Size.height + 2, m_Size.width + 2, mask.type());//延展图像
         mask.copyTo(Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));
         cv::floodFill(Temp, Point(0, 0), Scalar(255));
         Mat cutImg;//裁剪延展的图像
         Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);
         mask = mask | (~cutImg);
//         namedWindow("floodFill", 0);
//         imshow("floodFill", mask);
         mask = mask & thres_whole;
         _mask = mask.clone();
        }

     else if(enermy_color == 0){
        Mat thres_whole;
        vector<Mat> splited;
        Mat color,mask;
        int thres_max_color_red = 127;
        split(_roi,splited);
        cvtColor(_roi,thres_whole,CV_BGR2GRAY);
        threshold(thres_whole,thres_whole,60,255,THRESH_BINARY);
         //namedWindow("thresh_whole", 0);
         //imshow("thresh_whole", thres_whole);
        subtract(splited[2],splited[0], color);
        threshold(color, mask, thres_max_color_red,255,THRESH_BINARY);
        Size m_Size = mask.size();
        Mat Temp = Mat::zeros(m_Size.height + 2, m_Size.width + 2, mask.type());//延展图像
        mask.copyTo(Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));
        cv::floodFill(Temp, Point(0, 0), Scalar(255));
        Mat cutImg;//裁剪延展的图像
        Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);
        mask = mask | (~cutImg);
        mask = mask & thres_whole;
        _mask = mask.clone();
      }
    }

    else if(color_detection_mode == 2){     //inRange,已弃用
       if(enermy_color == 1){
           Mat hsv,mask;
           cvtColor(_roi, hsv, CV_BGR2HSV);
           inRange(hsv, Scalar(35, 0, 110), Scalar(110, 255, 255), mask);
           Mat kernel1 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
           Mat kernel2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
           morphologyEx(mask, mask, MORPH_CLOSE, kernel1);
           morphologyEx(mask, mask, MORPH_OPEN, kernel2);
           _mask = mask.clone();
        }

     else if(enermy_color == 0){
         Mat hsv,mask;
         cvtColor(_roi, hsv, CV_BGR2HSV);
         inRange(hsv, Scalar(5, 0, 110), Scalar(46, 255, 255), mask);
         Mat kernel1 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
         Mat kernel2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
         morphologyEx(mask, mask, MORPH_CLOSE, kernel1);
         morphologyEx(mask, mask, MORPH_OPEN, kernel2);
         _mask = mask.clone();
      }
    }

    else if(color_detection_mode == 3){     //红蓝通道相减,膨胀补偿
        Mat gray;
        Mat gray_binary,tempBinary, complement;
        // 灰度阈值二值
        cvtColor(_roi,gray,COLOR_BGR2GRAY);
        threshold(gray,gray_binary,50, 255,THRESH_BINARY);
        namedWindow("binary",WINDOW_NORMAL);
        imshow("binary",gray_binary);
        // 红蓝通道相减
        vector<Mat> splited;
        split(_roi,splited);
        if(enermy_color == 1)
        {

        //    erode(gray_binary,gray_binary,getStructuringElement(MORPH_RECT,Size(3,3)));
            subtract(splited[0],splited[2],tempBinary);
            threshold(tempBinary,tempBinary,60, 255,THRESH_BINARY);
            namedWindow("tempbinary",WINDOW_NORMAL);
            imshow("tempbinary",tempBinary);
        }
        else
        {
            subtract(splited[2],splited[0],tempBinary);
            threshold(tempBinary,tempBinary,B_SUB_R_LOW, B_SUB_R_UP,THRESH_BINARY);

        }

//        // 补充RGB中间过亮被排除的区域
//        if(distance > DISTANCE_BOUND)
//        {
//            dilate(tempBinary,tempBinary,getStructuringElement(MORPH_RECT,Size(5,5)));
//            threshold(gray,complement,230,255,THRESH_BINARY);
//            dilate(complement,complement,getStructuringElement(MORPH_RECT,Size(3,3)));
//            tempBinary = tempBinary | complement;
//        }

        // mask 操作
        _mask = tempBinary & gray_binary;
    }


    else if(color_detection_mode == 4)     //红蓝通道相减,膨胀补偿
    {
        if (enermy_color == 1)
        {
            vector<Mat>channels;
            Mat _grayImag, binBrightImg,gray,result,gray_binary;
//            double t1 = getTickCount();
            cvtColor(_roi,gray,COLOR_BGR2GRAY);
//            double t2 = getTickCount();
//            cout<<endl;
//            cout<<"-----CVTCOLOR-----"<<(t2 - t1)/getTickFrequency()*1000<<"ms"<<endl;
            threshold(gray,gray_binary,128, 255,THRESH_BINARY);
//            double t3 = getTickCount();
//            cout<<"-----THRESHOLD-----"<<(t3 -t2)/getTickFrequency()*1000<<"ms"<<endl;
//            namedWindow("gray",2);
//            imshow("gray",gray_binary);
            split(_roi,channels);
//            double t4 = getTickCount();
//            cout<<"-----SPLIT-----"<<(t4 - t3)/getTickFrequency()*1000<<"ms"<<endl;
            _grayImag = channels[0] - channels[2];
//            double t5 = getTickCount();
//            cout<<"-----SUBSTRACT-----"<<(t5 - t4)/getTickFrequency()*1000<<"ms"<<endl;
            threshold(_grayImag,binBrightImg,46,255,cv::THRESH_BINARY);
//            double t6 = getTickCount();
//            cout<<"-----THRESHOLD-----"<<(t6 - t5)/getTickFrequency()*1000<<"ms"<<endl;
            dilate(binBrightImg,binBrightImg,getStructuringElement(MORPH_RECT,Size(5,5)));
//            double t7 = getTickCount();
//            cout<<"-----DILATE-----"<<(t7 - t6)/getTickFrequency()*1000<<"ms"<<endl;
//            namedWindow("bin",WINDOW_NORMAL);
//            imshow("bin",binBrightImg);
            result = gray_binary & binBrightImg;
//            double t8 = getTickCount();
//            cout<<"-----RESULT-----"<<(t8 - t7)/getTickFrequency()*1000<<"ms"<<endl;
            cout<<endl;                     
            _mask = result.clone();
         }
        
        else if( enermy_color == 0)
        {
            vector<Mat>channels;
            Mat _grayImag, binBrightImg,gray,result,gray_binary;
    
            cvtColor(_roi,gray,COLOR_BGR2GRAY);
            threshold(gray,gray_binary,60, 255,THRESH_BINARY);
    //        namedWindow("gray",2);
    //        imshow("gray",gray_binary);
            split(_roi,channels);
            _grayImag = channels[2] - channels[0];
            threshold(_grayImag,binBrightImg,40,255,cv::THRESH_BINARY);
            dilate(binBrightImg,binBrightImg,getStructuringElement(MORPH_RECT,Size(7,7)));
            result = gray_binary & binBrightImg;
            _mask = result.clone();
        }

    }


   if(show_mask)
   {
       namedWindow("mask", WINDOW_NORMAL);
       imshow("mask", _mask);
   }
}


//筛选灯条
vector<vector<Point>> Armordetection::light_filter(bool show_light_filter_result)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hireachy;
    findContours(_mask, contours, hireachy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> contours_filter;
    RotatedRect rect;
    for(const vector<Point>& contour : contours)
    {
        rect=minAreaRect(contour);
        // k是旋转矩形的长宽比
        double k = min(rect.size.width, rect.size.height) / max(rect.size.width, rect.size.height);
        // 灯条的面积
        float lightContourArea = contourArea(contour);

        if(lightContourArea <10){/*cout<<"LIGHT:lightContourArea"<<lightContourArea<<endl*/;continue;}
        // 灯条面积太小的不要，注意这里可以再分细一点
        // 由于灯条刚刚转过来的时候，面积很小，w/h比也很小，可以结合这两个来筛选
        // 从而保证刚刚转过来的面积很小的灯条不会被筛选掉
        // 面积很小， w/h也要很小才保留
        if(lightContourArea < 280 && k > 0.4) {/* cout<<"LIGHT:lightContourArea"<<lightContourArea<<endl;*/ /*cout<<"1"<<endl;*/   continue;}
//        // 面积更小， w/h也要更小才保留
        if(lightContourArea < 47 && k > 0.3)  {/*cout<<"LIGHT:lightContourArea"<<lightContourArea<<endl;*/ /*cout<<"2"<<endl; */ continue;}

        // 灯条面积太大的不要
        if(lightContourArea > MAX_LightContourArea) {cout<<"area3"<<endl; continue;}

        // 旋转矩形长宽比太大的灯条不要  // 太胖的不要
        if( k > MAX_LightHighWeigh ) {cout<<"light4    "<<k<<endl; continue;}

        // 灯条左右倾斜超过一定角度的不要
        if (rect.size.width == min(rect.size.width, rect.size.height) &&
            fabs(rect.angle) > MAX_LightAngle1 ) {cout<<"anglr5"<<endl;continue;}
        if (rect.size.width == max(rect.size.width, rect.size.height) &&
            fabs(rect.angle) < MAX_LightAngle2 ) {cout<<"anglr6"<<endl; continue;}

        //灯条的凸度，即灯条面积比旋转矩形面积，对预处理要求较高，暂时不采用
        //if(lightContourArea /  rect.size.area() < 0.4) {/*cout<<"7"<<endl;*/ continue;}

        contours_filter.push_back(contour);
    }

    // 查看筛选后的灯条的结果
    if(show_light_filter_result)
    {
        if(contours_filter.size()>0){
        Mat output = Mat::zeros(_roi.size(), _roi.type());
        drawContours(output, contours_filter, -1, Scalar(255, 255, 255), -1);
        namedWindow("contours_filter", WINDOW_NORMAL);
        imshow("contours_filter", output);
        }
    }

    return contours_filter; //返回合格的灯条
}

void Armordetection::show_img()
{
    namedWindow("_img", WINDOW_NORMAL);
    imshow("_img", _img);
}


// 匹配灯条
vector<vector<Point>> Armordetection::light_match(vector<vector<Point>>& contours_filter, bool show_match_result)
{
    vector<vector<Point>> Match;  // 存放匹配好的灯条（总是两个一起连续存放）
    vector<int> Match_index;  //存放Match中灯条的索引（总是两个一起连续存放）
    int size = contours_filter.size();

    //灯条数大于2匹配才有意义，如果灯条数小于2，直接返回空的Match
    if (size >= 2){

    //按灯条中心x从大到小排序
    sort(contours_filter.begin(), contours_filter.end(), [](vector<Point>& light1,vector<Point>& light2)
          {  //Lambda函数,作为sort的cmp函数
        return Light(light1).get_center().x > Light(light2).get_center().x;});

    int temp[size][size] = {0};

    for(size_t i = 1; i < contours_filter.size(); i++)
    {

       Light light1 = Light(contours_filter.at(i));
       RotatedRect rect1=light1.get_rotatderect();
       // 灯条最上最下两个点
       Point top1 = light1.Top_point();
       Point bottom1 = light1.Bottom_point();
       double h1 = max(rect1.size.width, rect1.size.height); //灯条的长
       double w1 = min(rect1.size.width, rect1.size.height); //灯条的宽
        cout<<"---------------------------------------------------------------i"<<i<<endl;
       for(size_t j = 0; j < contours_filter.size(); j++)
       {
           if(i<=j) break; // 避免重复计算

           Light light2 = Light(contours_filter.at(j));
           RotatedRect rect2=light2.get_rotatderect();
           // 灯条最上最下两个点
           Point top2 = light2.Top_point();
           Point bottom2 = light2.Bottom_point();
           double h2 = max(rect2.size.width, rect2.size.height); //灯条的长
           double w2 = min(rect2.size.width, rect2.size.height); //灯条的宽

            cout<<"------------------j"<<j<<endl;
           //调节匹配条件的参数时，可以把一些条件注释掉，把单个匹配条件调到最优，再将所有匹配条件结合起来，增强鲁棒性
                cout<<" "<<endl;
           //由于灯条按x坐标由大到小排序，且i>j,所以light1是左灯条，light2是右灯条
           // 两个灯条是“ \ \ ” 时，旋转矩形的角度差不能太大
           if (rect1.size.width == w1 &&
               rect2.size.width == w2 &&
               fabs(rect1.angle - rect2.angle) < SUB_Angle1 )
           {  cout<<"1"<<endl;  cout<<"1角度差\\: "<<rect1.angle<<" "<<rect2.angle<<endl;
               temp[i][j] = temp[i][j] + 1; }
           // 两个灯条是“ / / ” 时，旋转矩形的角度差不能太大
           else if (rect1.size.width == h1 &&
                    rect2.size.width == h2 &&
                    fabs(rect1.angle - rect2.angle) < SUB_Angle2 )
           {  cout<<"2"<<endl; cout<<"2角度差//: "<<rect1.angle<<" "<<rect2.angle<<endl;
               temp[i][j] = temp[i][j] + 1; }
           // 两个灯条是“ \ / ” 时，旋转矩形的角度差不能太大
           else if (rect1.size.width == w1 &&
                    rect2.size.width == h2 &&
                    90 - fabs(rect1.angle - rect2.angle) < SUB_Angle3)
           {  cout<<"3"<<endl; cout<<"3角度差\/: "<<rect1.angle<<" "<<rect2.angle<<endl;
               temp[i][j] = temp[i][j] + 1; }
           // 两个灯条是“ / \ ” 时，旋转矩形的角度差不能太大
           else if (rect1.size.width == h1 &&
                    rect2.size.width == w2 &&
                    90 - fabs(rect1.angle - rect2.angle) < SUB_Angle4 )
           {  cout<<"4"<<endl;  cout<<"4角度差/\: "<<rect1.angle<<" "<<rect2.angle<<endl;
                temp[i][j] = temp[i][j] + 1; }
            else
               cout<<"no match 角度差"<<"angle1:"<<rect1.angle<<"angle2"<<rect2.angle<<endl;
           // 两个灯条的长度要差不多
           if ((double)min(h1, h2)/(double)max(h1, h2) > Lights_High)
           { cout<<"5灯条的长度"<<endl;  cout<<(double)min(h1, h2)/(double)max(h1, h2)<<endl;
             temp[i][j] = temp[i][j] + 1; }
           else
           cout<<"NOT_MATCH 灯条的长度"<<"min"<<min(h1, h2)<<"max"<<max(h1, h2)<<endl;
           // 两个灯条的中心点y坐标要差不多（这里只是粗略计算）
           if (fabs (top1.y+bottom1.y - top2.y-bottom2.y) <  3.2 * (double)(h1+h2)/(4))
           { cout<<"6"<<fabs (top1.y+bottom1.y - top2.y-bottom2.y)<<"\t"<<(double)(h1+h2)/(4)<<endl;
             cout<<"6: "<<fabs (top1.y+bottom1.y - top2.y-bottom2.y)<<"<"<<3.2*(double)(h1+h2)/(4)<<endl;
             temp[i][j] = temp[i][j] + 1; }



           // 两个灯条组成的旋转矩形长宽比要合理（这里只是粗略计算）
           double w = max(abs(top1.x-top2.x), abs(bottom1.x-bottom2.x));
           double h = max(abs(top1.y-bottom1.y), abs(top2.y-bottom2.y));
           if(w/h < U_Armoe_WH_Radio && w/h > L_Armoe_WH_Radio)
           {  cout<<"7旋转矩形长宽比:"<<endl;
              cout<<w/h<<endl; temp[i][j] = temp[i][j] + 1; }
           else
           {
              cout<<"7旋转矩形长宽比   NOT MATCH:"<<endl;
              cout<<w/h<<endl;
           }


           //两个灯条的顶点之间，底点之间连线的斜率不能太大
           double k1, k2;
           if ( top1.x-top2.x != 0)
           {
              k1 = fabs((double)(top1.y-top2.y)/(double)(top1.x-top2.x));
           }
           else {k1 = 10;} //用k1=10来表示斜率无穷大
           if ( bottom1.x-bottom2.x != 0)
           {
              k2 = fabs((double)(bottom1.y-bottom2.y)/ (double)(bottom1.x-bottom2.x));
           }
           else {k2 = 10;} //用k1=10来表示斜率无穷大
           if(k1 < Armor_Parallel_Radio && k2 < Armor_Parallel_Radio)
           { cout<<"8"<<endl;
             cout<<"k1:"<<k1<<"k2:"<<k2<<endl;temp[i][j] = temp[i][j] + 1; }
           else
           {
             cout<<"NOT MATCH"<<"k1:"<<k1<<"k2:"<<k2<<endl;
           }
       }
    }

    // 上面投票结果储存在temp中
       for(size_t i = 0; i < contours_filter.size(); i++)
       {
           for(size_t j = 0; j < contours_filter.size(); j++)
           {
               if(i<=j) break; // 避免重复计算

               //如果temp=5说明这索引为i，j的两个灯条已经满足了配对条件
               if(temp[i][j] == 5)
               {

                   //查找这两个灯条是否已经在Match向量当中
                   int c1 = count(Match.begin(), Match.end(), contours_filter[i]);
                   int c2 = count(Match.begin(), Match.end(), contours_filter[j]);

                   //如果两个都不在Match当中
//                       c1 = 0; c2 = 0;     //令c1,c2=0可以查看单靠以上八个条件的匹配效果
                   if(c1 == 0 && c2 ==0)
                   {
                       //把这两个灯条放进Match，并把他们的索引号i，j放进Match_index
                       Match.push_back(contours_filter.at(i));
                       Match.push_back(contours_filter.at(j));
                       Match_index.push_back(i);
                       Match_index.push_back(j);
                   }


                  // 如果两个灯条至少有一个已经被放进Match，说明有一个灯条同时与两个灯条配对了
                  // 解决思路是找出这两对灯条，再匹配一次，提出更严格的条件，最后只保留最优的那一对
                  else
                   {
                        for(size_t m = 0; m < Match_index.size(); m++)
                        {
                            int index1, index2, m1, m2;
                            vector<int>::iterator it;
                            vector<vector<Point>>::iterator it2;

                            // 如果第i或j个灯条之前被放进Match过
                            if (i == Match_index.at(m) || j == Match_index.at(m))
                            {
                               // index1，index2是 第i(j)个灯条 和第i(j)个灯条匹配的灯条 的索引
                               // 因为匹配的灯条总是一对一对放进Match的，所以要看m的奇偶
                               if(m%2 == 0)
                               {m1 = m; m2 = m+1; index1 = Match_index.at(m); index2 = Match_index.at(m+1);}
                               else {m1 = m-1; m2 = m; index1 = Match_index.at(m-1); index2 = Match_index.at(m);}

                               //现在要看的是i，j这一对灯条和index1，index2这对灯条哪个更加匹配，就保留哪一对

                               // 现在的条件是哪对灯条中心点的欧氏距离更小哪对就最优
                               // 单用这个条件效果就已经很好了
                               // 后期如果需要可以再加入灯条角度等条件
                               Light light_i = Light(contours_filter.at(i));
                               Light light_j = Light(contours_filter.at(j));
                               double distance_ij = abs(light_i.get_center().x - light_j.get_center().x)+
                                                    abs(light_i.get_center().y - light_j.get_center().y);
//                               cout<<"distance_ij"<<distance_ij<<endl;
                               Light light_index1 = Light(contours_filter.at(index1));
                               Light light_index2 = Light(contours_filter.at(index2));
                               double distance_index = abs(light_index1.get_center().x - light_index2.get_center().x) +
                                                       abs(light_index1.get_center().y - light_index2.get_center().y);
//                               cout<<"distance_index"<<distance_index<<endl;


                               Point top1 = light_i.Top_point() + _now_roi.tl();
                               Point bottom1 = light_i.Bottom_point() + _now_roi.tl();
                               Point top2 = light_j.Top_point() + _now_roi.tl();
                               Point bottom2 = light_j.Bottom_point() + _now_roi.tl();

                               vector<Point> vPolygonPoint;
                               vPolygonPoint.push_back(top1);
                               vPolygonPoint.push_back(bottom1);
                               vPolygonPoint.push_back(bottom2);
                               vPolygonPoint.push_back(top2);

                               //画旋转矩形
                               RotatedRect armor_rect = minAreaRect(vPolygonPoint);
                               Point2f vertex[4];
                               armor_rect.points(vertex);
                               for (int j = 0; j < 4; j++)
                               {
                                   line(_img, vertex[j], vertex[(j + 1) % 4], Scalar(0, 255, 255),4, CV_AA);
                               }
                               top1 = light_index1.Top_point() + _now_roi.tl();
                               bottom1 = light_index1.Bottom_point() + _now_roi.tl();
                               top2 = light_index2.Top_point() + _now_roi.tl();
                               bottom2 = light_index2.Bottom_point() + _now_roi.tl();
                               vPolygonPoint.clear();
                               vPolygonPoint.push_back(top1);
                               vPolygonPoint.push_back(bottom1);
                               vPolygonPoint.push_back(bottom2);
                               vPolygonPoint.push_back(top2);

                               //画旋转矩形
                               armor_rect = minAreaRect(vPolygonPoint);
                               armor_rect.points(vertex);
                               for (int j = 0; j < 4; j++)
                               {
                                   line(_img, vertex[j], vertex[(j + 1) % 4], Scalar(0, 255, 255), 4, CV_AA);
                               }






                               if(distance_index > distance_ij)
                               {

                                   it=Match_index.begin();
                                   Match_index.erase(it+m1);
                                   Match_index.erase(it+m1);
                                   it2=Match.begin();
                                   Match.erase(it2+m1);
                                   Match.erase(it2+m1);

                                   Match.push_back(contours_filter.at(i));
                                   Match.push_back(contours_filter.at(j));
                                   Match_index.push_back(i);
                                   Match_index.push_back(j);
                               }
                               break;
                            }
                        }
                   }

               }
           }
       }
     }

    if (show_match_result){
        if(Match.size()>=2){
        Mat output=Mat::zeros(_roi.size(), _roi.type());
        drawContours(output, Match, -1, Scalar(255, 255, 255), -1);
        namedWindow("contours_match", WINDOW_NORMAL);
        imshow("contours_match", output);
        }
    }

    return Match;
}


//在找到的所有装甲板中选择最优的打击目标
//要确保Match.size>=2才能使用这个函数
vector<Point> Armordetection::choose_target(vector<vector<Point>>& Match)
{
    vector<int> s;
    vector<RotatedRect> rr;
    vector<vector<Point>> armor_4point;
    vector<vector<Point>> target_two_light;
    int ii, ind=0;

    for(size_t i = 0; i<Match.size(); i=i+2)
    {
        Light light1 = Light(Match.at(i));
        Light light2 = Light(Match.at(i+1));
        Point top1 = light1.Top_point() + _now_roi.tl();
        Point bottom1 = light1.Bottom_point() + _now_roi.tl();
        Point top2 = light2.Top_point() + _now_roi.tl();
        Point bottom2 = light2.Bottom_point() + _now_roi.tl();

        vector<Point> vPolygonPoint;
        vPolygonPoint.push_back(top1);
        vPolygonPoint.push_back(bottom1);
        vPolygonPoint.push_back(bottom2);
        vPolygonPoint.push_back(top2);

        //画旋转矩形
        RotatedRect armor_rect = minAreaRect(vPolygonPoint);
        Point2f vertex[4];
        armor_rect.points(vertex);
        for (int j = 0; j < 4; j++)
        {
            line(_img, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
        }

        s.push_back(armor_rect.size.width * armor_rect.size.height); //保存旋转矩形面积
        rr.push_back(armor_rect);          // 保存旋转矩形
        armor_4point.push_back(vPolygonPoint);   // 保存一对灯条的四个顶点
        target_two_light.push_back(Match.at(i));      // 保存一对灯条
        target_two_light.push_back(Match.at(i+1));
     }

    //找出面积最大的旋转矩形的索引
    for(ii=1; ii<s.size(); ii++)
    {
        if(s[ii]>s[ind])
        {
            ind = ii;
        }
    }
   circle(_img, rr.at(ind).center, 8, Scalar(0,0,255), -1);
   _target_two_light.push_back(target_two_light.at(2*ind));
   _target_two_light.push_back(target_two_light.at(2*ind+1));

   return armor_4point.at(ind);
}

Rect Armordetection::get_roi(vector<Point>& armor_4point, double roi_scale)
{

   int xmax =0, xmin = 10000, ymax = 0, ymin = 10000, rw=0, rh=0;  //初始化一下

    Point top11 = armor_4point.at(0);
    Point bottom11 = armor_4point.at(1);
    Point top22 = armor_4point.at(3);
    Point bottom22 = armor_4point.at(2);

    int xmin1 = min(top11.x, top22.x);
    int xmin2 = min(bottom11.x, bottom22.x);
    xmin = min(xmin1, xmin2);

    int ymin1 = min(top11.y, top22.y);
    int ymin2 = min(bottom11.y, bottom22.y);
    ymin = min(ymin1, ymin2);

    int xmax1 = max(top11.x, top22.x);
    int xmax2 = max(bottom11.x, bottom22.x);
    xmax = max(xmax1, xmax2);

    int ymax1 = max(top11.y, top22.y);
    int ymax2 = max(bottom11.y, bottom22.y);
     ymax = max(ymax1, ymax2);

    xmin = max(1, (int)(xmin - 2*(xmax-xmin)*roi_scale));
    ymin = max(1, (int)(ymin - 2*(ymax-ymin)*roi_scale));

    rw = min((int)(xmax - xmin +(xmax-xmin)*roi_scale), _img.cols-xmin);
    rw = max(1, rw);
    rh = min((int)(ymax - ymin +(ymax-ymin)*roi_scale), _img.rows-ymin);
    rh = max(1, rh);

    Rect roi_rect = Rect(xmin, ymin, rw, rh);

    return roi_rect;
}
void Armordetection:: put_text_into_img(VisionData &vdata/*, main_information &info*/)
{
    //设置绘制文本的相关参数
    string text1 = "distance: "+ to_string(vdata.dis.f);
    string text2 = "send_pitch: " + to_string(vdata.pitch_angle.f);
    string text3 = "send_yaw: " + to_string(vdata.yaw_angle.f);
//    string text4 = "get_yaw: " + to_string(info.now_frame.car_yaw_angel);
//    string text5 = "get_pitch: " + to_string(info.now_frame.car_pitch_angel);
//    string text6 = "yaw_angel_speed: " + to_string(info.now_frame.yaw_angel_speed);
//    string text7 = "pitch_angel_speed: " + to_string(info.now_frame.pitch_angel_speed);
//    string text8 = "object_speed: " + to_string(info.now_frame.object_speed);
    int font_face = FONT_HERSHEY_COMPLEX;
    double font_scale = 1;
    int thickness = 2;
    int baseline;

    //获取文本框的长宽
    Size text1_size = getTextSize(text1, font_face, font_scale, thickness, &baseline);
    //将文本框居中绘制
    Point origin;
    origin.x = 20;
    origin.y = text1_size.height*1;
    putText(_img, text1, origin, font_face, font_scale, Scalar(255, 0, 0), thickness, 8, 0);
    origin.y = text1_size.height * 3;
    putText(_img, text2, origin, font_face, font_scale, Scalar(255, 0, 0), thickness, 8, 0);
    origin.y = text1_size.height * 5;
    putText(_img, text3, origin, font_face, font_scale, Scalar(255, 0, 0), thickness, 8, 0);
//    origin.y = text1_size.height * 7;
//    putText(_img, text4, origin, font_face, font_scale, Scalar(0, 255, 0), thickness, 8, 0);
//    origin.y = text1_size.height * 9;
//    putText(_img, text5, origin, font_face, font_scale, Scalar(0, 255, 0), thickness, 8, 0);
//    origin.y = text1_size.height * 11;
//    putText(_img, text6, origin, font_face, font_scale, Scalar(0, 255, 0), thickness, 8, 0);
//    origin.y = text1_size.height * 13;
//    putText(_img, text7, origin, font_face, font_scale, Scalar(0, 255, 0), thickness, 8, 0);
//    origin.y = text1_size.height * 15;
//    putText(_img, text8, origin, font_face, font_scale, Scalar(0, 255, 0), thickness, 8, 0);
}
// 获取平移矩阵
Mat Armordetection::get_tVec(vector<Point> armor_4point, Mat c,  Mat d, double h, double w)
{
   Point top11 = armor_4point.at(0);
   Point bottom11 = armor_4point.at(1);
   Point top22 = armor_4point.at(3);
   Point bottom22 = armor_4point.at(2);

   circle(_img, top11, 4, Scalar(0,0,255), -1);
   putText(_img, "top1", top11, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
   circle(_img,  bottom11, 4, Scalar(0,0,255), -1);
   putText(_img, "bottom1", bottom11, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
   circle(_img, top22, 4, Scalar(0,0,255), -1);
   putText(_img, "top2", top22, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
   circle(_img, bottom22, 4, Scalar(0,0,255), -1);
   putText(_img, "bottom2", bottom22, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));

   //自定义的物体世界坐标，单位为mm
   vector<Point3f> obj=vector<Point3f>
   {
   Point3f(-w, -h, 0),	//tl
   Point3f(w, -h, 0),	//tr
   Point3f(w, h, 0),	//br
   Point3f(-w, h, 0)	//bl
   };
   vector<Point2f> pnts=vector<Point2f>
   {
   Point2f(top11),	    //tl
   Point2f(top22),	    //tr
   Point2f(bottom22),	//br
   Point2f(bottom11)	//bl
   };

   Mat rVec = Mat::zeros(3, 1, CV_64FC1); //init rvec
   Mat tVec = Mat::zeros(3, 1, CV_64FC1); //init tvec
   //进行位置解算
   solvePnP(obj,pnts,c,d,rVec,tVec,false, SOLVEPNP_ITERATIVE);
   distance = sqrt(tVec.at<double>(0)*tVec.at<double>(0)+tVec.at<double>(1)*tVec.at<double>(1)+
                   tVec.at<double>(2)*tVec.at<double>(2));

   yaw_angle = atan(tVec.at<double>(0)/tVec.at<double>(2)) *180 /3.14;
   pitch_angle = -atan(tVec.at<double>(1)/tVec.at<double>(2)) *180 / 3.14;
//TODO: edit by liuhongjie
//   string str_d = (string)("distances: ")+ to_string(distance);
//   string str_r1 = (string)("pitch: ")+ to_string(-pitch_angle*180/3.14);
//   string str_r2 = (string)("yaw: ")+ to_string(-yaw_angle*180/3.14);
//   putText(_img, str_d, Point(150, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
//   putText(_img, str_r1, Point(10, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
//   putText(_img, str_r2, Point(310, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));

//TODO: edit by liuhongjie
   return tVec;
}

//Mat Armordetection::get_num_img(int img_size, bool show_num_img)
//{
//   Light light1 = _target_two_light.at(0);
//   Light light2 = _target_two_light.at(1);

//   // 保证light1是左灯条
//   if(light1.get_center().x > light2.get_center().x)
//   {
//       light1 = Light(_target_two_light.at(1));
//       light2 = Light(_target_two_light.at(0));
//   }

//   Mat Matrix, num_img;
//   Point2f sourcePoints[4], objectPoints[4];

//   double k1 = light1.line_k();
//   int light1_height = abs(light1.Top_point().y - light1.Bottom_point().y);
//   Point  p1 = light1.Right_point() + _now_roi.tl();
//   int y11 = light1.Top_point().y - 0.6 * light1_height + _now_roi.tl().y;
//   int y12 = light1.Bottom_point().y + 0.6 * light1_height + _now_roi.tl().y;
//   int x11 = (y11 - p1.y)/k1 + p1.x;
//   int x12 = (y12 - p1.y)/k1 + p1.x;

//   double k2 = light2.line_k();
//   int light2_height = abs(light2.Top_point().y - light2.Bottom_point().y);
//   Point  p2 = light2.Left_point()+ _now_roi.tl();
//   int y21 = light2.Top_point().y - 0.6 * light2_height + _now_roi.tl().y;
//   int y22 = light2.Bottom_point().y + 0.6 * light2_height + _now_roi.tl().y;
//   int x21 = (y21 - p2.y)/k1 + p2.x;
//   int x22 = (y22 - p2.y)/k1 + p2.x;

//   // 保证以上四个点的坐标不会超出图片的范围
//   x11 = max(1, x11);
//   x11 = min(_img.cols, x11);
//   y11 = max(1, y11);
//   y11 = min(_img.rows, y11);

//   x21 = max(1, x21);
//   x21 = min(_img.cols, x21);
//   y21 = max(1, y21);
//   y21 = min(_img.rows, y21);

//   x12 = max(1, x12);
//   x12 = min(_img.cols, x12);
//   y12 = max(1, y12);
//   y12 = min(_img.rows, y12);

//   x22 = max(1, x22);
//   x22 = min(_img.cols, x22);
//   y22 = max(1, y22);
//   y22 = min(_img.rows, y22);

//   sourcePoints[0].x = x11; sourcePoints[0].y = y11; //left_top
//   sourcePoints[1].x = x21; sourcePoints[1].y = y21;  //right_top
//   sourcePoints[2].x = x12; sourcePoints[2].y = y12;   //left_bottom
//   sourcePoints[3].x = x22; sourcePoints[3].y = y22;  //right_bottom

//   objectPoints[0].x = 0; objectPoints[0].y = 0;
//   objectPoints[1].x = img_size; objectPoints[1].y = 0;
//   objectPoints[2].x = 0; objectPoints[2].y = img_size;
//   objectPoints[3].x = img_size; objectPoints[3].y = img_size;

//   Matrix = getPerspectiveTransform(sourcePoints, objectPoints);
//   warpPerspective(_frame, num_img, Matrix, Size(img_size, img_size), INTER_LINEAR);

//  if(show_num_img){
//       namedWindow("num_img", WINDOW_NORMAL);
//       imshow("num_img", num_img);
//       circle(_img, sourcePoints[0], 4, Scalar(0,0,255), -1);
//       circle(_img, sourcePoints[1], 4, Scalar(0,0,255), -1);
//       circle(_img, sourcePoints[2], 4, Scalar(0,0,255), -1);
//       circle(_img, sourcePoints[3], 4, Scalar(0,0,255), -1);
//   }

//   return num_img;
//}


Mat Armordetection::get_num_img(int img_size, bool show_num_img)
{
#ifdef TEST_TIME
    auto start=getTickCount();
#endif
   Light light1 = _target_two_light.at(0);
   Light light2 = _target_two_light.at(1);

   // 保证light1是左灯条
   if(light1.get_center().x > light2.get_center().x)
   {
       light1 = Light(_target_two_light.at(1));
       light2 = Light(_target_two_light.at(0));
   }

   Mat Matrix, num_img;
   Point2f sourcePoints[4], objectPoints[4];

   double k1 = light1.line_k();
   int light1_height = abs(light1.Top_point().y - light1.Bottom_point().y);
   Point  p1 = light1.Right_point() + _now_roi.tl();
   int y11 = light1.Top_point().y - 0.3 * light1_height + _now_roi.tl().y;
   int y12 = light1.Bottom_point().y + 0.6 * light1_height + _now_roi.tl().y;
   int x11 = (y11 - p1.y)/k1 + p1.x;
   int x12 = (y12 - p1.y)/k1 + p1.x;

   double k2 = light2.line_k();
   int light2_height = abs(light2.Top_point().y - light2.Bottom_point().y);
   Point  p2 = light2.Left_point()+ _now_roi.tl();
   int y21 = light2.Top_point().y - 0.3 * light2_height + _now_roi.tl().y;
   int y22 = light2.Bottom_point().y + 0.6 * light2_height + _now_roi.tl().y;
   int x21 = (y21 - p2.y)/k1 + p2.x;
   int x22 = (y22 - p2.y)/k1 + p2.x;

   // 保证以上四个点的坐标不会超出图片的范围
   x11 = max(1, x11);
   x11 = min(_img.cols, x11);
   y11 = max(1, y11);
   y11 = min(_img.rows, y11);

   x21 = max(1, x21);
   x21 = min(_img.cols, x21);
   y21 = max(1, y21);
   y21 = min(_img.rows, y21);

   x12 = max(1, x12);
   x12 = min(_img.cols, x12);
   y12 = max(1, y12);
   y12 = min(_img.rows, y12);

   x22 = max(1, x22);
   x22 = min(_img.cols, x22);
   y22 = max(1, y22);
   y22 = min(_img.rows, y22);

   sourcePoints[0].x = x11; sourcePoints[0].y = y11; //left_top
   sourcePoints[1].x = x21; sourcePoints[1].y = y21;  //right_top
   sourcePoints[2].x = x12; sourcePoints[2].y = y12;   //left_bottom
   sourcePoints[3].x = x22; sourcePoints[3].y = y22;  //right_bottom

   objectPoints[0].x = 0; objectPoints[0].y = 0;
   objectPoints[1].x = img_size; objectPoints[1].y = 0;
   objectPoints[2].x = 0; objectPoints[2].y = img_size;
   objectPoints[3].x = img_size; objectPoints[3].y = img_size;

   Matrix = getPerspectiveTransform(sourcePoints, objectPoints);
   warpPerspective(_frame, num_img, Matrix, Size(img_size, img_size), INTER_LINEAR);


//   Mat gamma_num;
//   gamma_num=num_pre->gamma_trans(num_img);
    if(show_num_img){
        namedWindow("_now_roi",WINDOW_NORMAL);
        cv::imshow("_now_roi",num_img);
       circle(_img, sourcePoints[0], 4, Scalar(0,0,255), -1);
       circle(_img, sourcePoints[1], 4, Scalar(0,0,255), -1);
       circle(_img, sourcePoints[2], 4, Scalar(0,0,255), -1);
       circle(_img, sourcePoints[3], 4, Scalar(0,0,255), -1);
    }


#ifdef TEST_TIME
    auto end=getTickCount();
    cout<<"Armordetection::get_num_img consuming time : "<<(end-start)/getTickFrequency()<<endl;
#endif

   return num_img;
}

void Armordetection::begin_to_detect(Mat c, Mat d, double roi_scale, double armorH, double armorW,
                     bool use_roi, int color_detection_mode, bool show_mask,
                     bool show_light_filter_result, bool show_match_result,
                     bool get_num_image, int num_img_size, bool show_num_img,Numpredict &num_pre)
{
    double t = (double)getTickCount();
    num_pred=&(num_pre);
    vector<vector<Point>> light_filter, match;
    vector<Point> target;


    this->set_img(use_roi);
    double t2 = (double)getTickCount();
    this->color_detection(color_detection_mode, show_mask);
    double t3 = (double)getTickCount();
    light_filter = this->light_filter(show_light_filter_result);
    double t4 = (double)getTickCount();
    match = this->light_match(light_filter, show_match_result);
    double t5 = (double)getTickCount();
    // 如果有识别到目标
    if(match.size()>=2)
    {
     _flag = 1;     // 目前先用1来表示检测到装甲板，等数字识别搞定了，再细分，用1，2区分大小装甲板
    target = this->choose_target(match);  // 打击目标的四个顶点
//    cout<<55555<<endl;
    double t6 = (double)getTickCount();
    _rotaterect = minAreaRect(target);
    _next_roi = this->get_roi(target, roi_scale);
    double t7 = (double)getTickCount();
    rectangle(_img, _now_roi, Scalar(0, 255, 0), 2, LINE_AA, 0);
    _tVec = this->get_tVec(target, c, d, armorH, armorW);
    double t8 = (double)getTickCount();



    if(get_num_image)
    {
        auto start_num=getTickCount();

        _num_img = this->get_num_img(num_img_size, show_num_img);
        int number_index=num_pred->predict_project(_num_img,true);
//        cout<<"number index: "<<number_index<<endl;

        auto end_num=getTickCount();
//        cout<<"predict num consuming time : "<<(end_num-start_num)/getTickFrequency()<<endl;
     }





    }
    else   //设置ROI
    {
        rectangle(_img, _now_roi, Scalar(0, 255, 0), 2, LINE_AA, 0);
        if (_lossFrameK < 10) {

            if (_lossFrameK> 0)
            {
                roi_scale = 0.1*roi_scale + 0.01 * (double)_lossFrameK;
            }

             int xmin = _now_roi.tl().x;
             int ymin = _now_roi.tl().y;
             int xmax = _now_roi.br().x;
             int ymax = _now_roi.br().y;

              xmin = max(1, (int)(xmin - 2*(xmax-xmin)*roi_scale));
              ymin = max(1, (int)(ymin - 2*(ymax-ymin)*roi_scale));

              int rw = min((int)(xmax - xmin +(xmax-xmin)*roi_scale * 2), _img.cols-xmin);
              rw = max(1, rw);
              int rh = min((int)(ymax - ymin +(ymax-ymin)*roi_scale*0.5), _img.rows-ymin);
              rh = max(1, rh);

               _next_roi = Rect(xmin, ymin, rw, rh);
          }

        else
         {
              _next_roi = Rect(0, 0, _img.cols, _img.rows);
         }
    }


#ifdef TIME_DEBUG
//    cout<<"============ROI============: "<<(t2 - t)/getTickFrequency()*1000<<"ms"<<endl;
//    cout<<"============COLOR_DETECTION============="<<(t3 - t2)/getTickFrequency()*1000<<"ms"<<endl;
//    cout<<"============LIGHT_FILTER==============="<<(t4 - t3)/getTickFrequency()*1000 <<"ms" <<endl;
//    cout<<"=============LIGHT_MATCH==============="<<(t5 - t4)/getTickFrequency()*1000<<"ms"<<endl;
//    cout<<"============CHOOSE_TARGET================"<<(t6 - t5)/getTickFrequency()*1000<<"ms"<<endl;
//    cout<<"============GET_ROI=================="<<(t7 - t6)/getTickFrequency()*1000<<"ms"<<endl;
//    cout<<"============GET_TVEC============="<<(t8 -t7)/getTickFrequency()*1000<<"ms"<<endl;
    // 计算帧率
//    _dt = ((double)getTickCount() - t) / (double)getTickFrequency();
//    cout<<"==================TIME=================="<<_dt*1000<<endl;
//    _fps = 1/_dt;
//    string fpsString = (string)("fps: ")+ to_string(int(_fps + 0.5));
//    putText(_img, fpsString, Point(20, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
//    cout<<"fps"<<_fps<<endl;
#endif

}

