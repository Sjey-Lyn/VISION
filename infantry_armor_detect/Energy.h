#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <rp_kalman.h>
using namespace cv;//这是个不好的编程习惯，尽量改用cv::
//using cv::Mat;
//using cv::RotatedRect;
//using cv::Point2f;
//using cv::Point;
//using cv::Rect;
//using cv::Scalar;
//using cv::min;
//using cv::max;


extern RotatedRect center_R;
extern const double PI;
extern int SCREEN_MAX_X;//屏幕分辨率1280x720
extern int SCREEN_MAX_Y;

class RMvector
{
	// 待优化
	// 这个应该是可以改成Point2f的派生类，这个想法有点欠妥，xy冲突
public:
	double x;							// 以原点为中心的x坐标
	double y;							// 以原点为中心的y坐标
	double angle;						// 向量的角度，以横向为基准
	RMvector()
	{
		x = 0; y = 0;
		angle = 0;
	}
	RMvector(Point2f& start, Point2f& end)
	{
		Point2f vec = end - start;
		x = vec.x; y = vec.y;
		angle = atan(y / x);// 误差的可能来源
	}
	RMvector(double a)// 由角度计算单位向量
	{
		x = cos(a * PI / 180); y = sin(a * PI / 180);
		angle = a; // 可能超出0-360范围，不安全，建议改为mod
	}
	double getNorm()// 计算向量的长度
	{
		return sqrt(x * x + y * y);
	}
	double dot(RMvector& new_vector)// 向量点乘另外一个向量
	{
		return x * new_vector.x + y * new_vector.y;
	}
	double cross(RMvector& new_vector)// 向量叉乘另外一个向量，注意顺序
	{
		return x* new_vector.y - y * new_vector.x;
	}
};

class ArmorRect
{
	// 这个可以改成旋转矩形的派生类
public:
	Point2f center;					// 旋转矩形的中心
	double width;					// 旋转矩形的width
	double height;					// 旋转矩形的height
	double long_edge;				// 长边
	double short_edge;				// 短边
	double rotatedrect_angle;		// 旋转矩形原有的角度(-90, 0]
	double angle;					// 装甲板的角度，[0, 180);	//由这两个角度就可以计算切线和法线的向量了
	double tangent_angle;			// 装甲板切线角度
	double area;					// 面积
	RMvector to_center;				// 向心向量
	RMvector tangent;				// 切线
	double speed;					// 装甲板的角速度，每帧多少rad
	bool hitted;					// 该装甲板是否被点亮标志位 // 外部接口
	RMvector vertex2center[4];
	Point2f vertices[4];
	Point2f fan_center;				// 大扇叶的中心
	double eps = 0.35;				// 速度阈值， 小于这个阈值速度视为0
	int minErrorIndex = -1;

	ArmorRect() {};
	ArmorRect(RotatedRect& rect)
	{
		center = rect.center;
		width = rect.size.width;
		height = rect.size.height;
		long_edge = max(width, height);
		short_edge = min(width, height);
		rotatedrect_angle = rect.angle;
		area = width * height;
		setAngel();
		setTangent();
		speed = 0;
		hitted = 0;
		rect.points(vertices);
	}
	void setParam(RotatedRect& rect)// 用于更新新的装甲板
	{
		center = rect.center;
		width = rect.size.width;
		height = rect.size.height;
		long_edge = max(width, height);
		short_edge = min(width, height);
		rotatedrect_angle = rect.angle;
		area = width * height;
		setAngel(); // 设置角度
		setTangent();
		speed = 0;
		hitted = 0;
		rect.points(vertices);
	}
	void setSpeed(double w)
	{
		if (w < eps) // 解决顺逆判断错误问题 /// 这个或许不妥
			speed = 0;
		speed = w;
	}
	void setAngel()
	{// 已稳
		angle = width > height ? (rotatedrect_angle + 90) :  rotatedrect_angle;
	}
	void setTangent()
	{// 已稳
		tangent_angle = width < height ? (rotatedrect_angle + 90) : rotatedrect_angle;
	}
	void setToCenter()
	{		
		for (int i = 0; i < 4; i++)
		{
			RMvector vec1(vertices[i], center_R.center);
			vertex2center[i] = vec1; // 可以直接赋值，值传递
		}
		RMvector vec(center, center_R.center);// 这里可以重载()运算符来简化代码，一个小优化，不是必要
		to_center = vec;
	}
};

class RMarray
{
private:
	int len;
public:
	std::vector<double> data; // 如果可以写成类模板形式，可以但没必要
	RMarray() 
	{
		data.push_back(0);// 防止出bug
	}
	RMarray(int length) 
	{
		if (length == 0)
		{
			data.push_back(0);// 防止出bug
		}
		len = length;
		for (int i = 0; i < len; i++)
		{
			data.push_back(0);
		}
	}
	RMarray(std::vector<double> &data_flow)
	{
		if(data_flow.size() == 0)
			data.push_back(0);
		len = data_flow.size();
		for (int i = 0; i < len; ++i)
		{
			data.push_back(data_flow[i]);
		}
	}
	RMarray(std::queue<double>& data_flow)
	{
		if (data_flow.size() == 0)
			data.push_back(0);
		len = data_flow.size();
		for (int i = 0; i < len; ++i)
		{
			double temp = data_flow.front();
			data.push_back(temp);
			data_flow.pop();
			data_flow.push(temp);
		}
	}
	//void clear()
	//{
	//	len = 0;
	//	data.clear();// 长度置0，但不回收内存
	//	data.push_back(0);
	//	data.swap(R())
	//}
	double& operator[](int i)
	{
		//if (i >= len)
		//{
		//	//cout << "索引超过最大值" << endl;
		//	// 应该throw error
		//	return ;
		//}
		return data[i];
	}
	int size()
	{
		return len;
	}
	double conv(RMarray& arr)
	{
		// 如果该函数需要复用，则多加一个string参数：full,same...
		// 这里只实现same方法
		if (arr.size() > len)
		{
			//error
			return 0;
		}
		int arrLen = arr.size();
		std::vector<double> vec;
		double sum = 0;
		for (int i = 0; i < len - arrLen + 1; i++) // 内积这么多次
		{
			for (int j = 0; j < arrLen; j++)
			{
				vec[i] = data[i + j] * arr[arrLen - 1 - (i + j)];
			}
		}
		for (auto& v : vec)
		{
			sum += v;
		}
		//return sum / vec.size();
		return sum;
	}
	double dot(RMarray& arr)
	{
		if (arr.size() != len)
		{
			// throw error
			return 0;
		}
		double sum = 0;
		for (int i = 0; i < len; i++)
		{
			sum += arr[i] * data[i];
		}
		return sum;
	}
};

// 此类主要是对能量机关输入的两个装甲板类来做处理
// 一个是当前的装甲板，一个是前一个装甲板
class Energy
{
private:
	double TA = 0.785;
	double TC = 1.305;
	double TW = 0.0384;
	int BIAS = -60;
	int THRESH_BR = 56;
	int THRESH_GRAY = 79;
	int MIN_AREA = 200;						// 所有可识别物的最小接受面积
	int IS_RED = 0;							// 敌方装甲板颜色为红色则为1
	int SHUN = 0;							// 判断是顺逆时针，0检测不出，1为顺，2为逆
	double NEWSPEED;						// 装甲板的新速度
	kalman_filter kf;						// 定义卡尔曼滤波器
	double former_speed;					// 前一帧的速度
	double second_to_frame = 0.03;			// 一帧多少秒
	double bias = 0;						// 扰动的大小
    int DILATE_EOFF = 9;
	double radius = 3;
//	int pass_frames = 10;
//	int pass_index = 0;
	std::vector<double> initBigSpeed;
	std::queue<double> rune_measure_speed;	// 速度队列
	//std::vector<double> rune_measure_speed;	// 速度队列
    int speed_max_num = 300;				// 速度队列元素的最大长度
	int begin_bigPredcit_time = 0;
	double meanSpeed;
	int valid_speed_num = 0;
	// 上次调车的时候发现会由掉帧的问题，可以设置ROI跟踪圆心
	// 先实现加速度大符扰动预测
public:
	// 需要返回给电控的值
	float pre_x = 0;						// 预测击打的x坐标
	float pre_y = 0;						// 预测击打的y坐标
	int is_find = 0;						// 是否找到装甲板
	// 偏移时间
	float getTime;
	int time2fire = 0;
	bool is_big = 0;
	Energy() 
	{
		kf.init_kalman_filter(1);				// 卡尔曼滤波速度初始化
	}
	// 需要一些拷贝构造函数
	// 通过每一帧图像得到一个mask。图像识别主要函数
	void videoProcess(Mat& frame, Mat& mask, ArmorRect& present, ArmorRect& former)
	{
		getMask(frame, mask);					// 得到掩膜
		/* ------从mask中找出轮廓确定present------- */
		getCoutour(mask, frame, present);		// 找装甲板的轮廓 // 得到新的present
		getR(mask, present, former);					// 得到中心R	// 圆心需要装甲板来确定 // 确定装甲版的向心向量
		// 只有完成前面两步操作才可以确定present
		present.setToCenter();
		if (is_find)
		{
			judgeDirect(present, former, frame);	// 判断顺逆 
			//newSpeedMeasure(present, former);		// 暂不使用，优化用(经测试，不能使用)
			getSpeed(present, former, frame);		// 计算角速度
			former = present;						// 保留装甲版信息
			drawCircle(frame, present);
		}
	}
	void reset()
	{
		//std::cout << pre_y << std::endl;//debug用
		pre_x = 0;
		pre_y = 0;
	}
	// 外部接口:设置阈值
	void setThresh(int br, int gray) {
		THRESH_BR = br;
		THRESH_GRAY = gray;
	}
	// 外部接口:是否为红色
	void isRed(bool b)
	{
		IS_RED = b;
	}
	
	void getMask(Mat& frame, Mat& mask);
	// 借助当前帧的装甲板和之前帧的装甲板计算角速度或者是角度，主要用到点乘
	void getSpeed(ArmorRect& present, ArmorRect& former, Mat& frame);
	// 预测需要击打的位置，需要返回pitch和yaw
	void predict(ArmorRect& present, Mat& frame);
	// 判断是顺时针还是逆时针
	void judgeDirect(ArmorRect& present, ArmorRect& former, Mat& frame);
	// 通过轮廓查找的方法找到装甲板的位置
	void getCoutour(Mat& mask, Mat& frame, ArmorRect& armor_center);
	// 这个还可以调调，识别R中心时的一系列判断
	bool isValidCenterRContour(const std::vector<Point>& center_R_contour);
	// 得到R中心的函数，得到center_R的坐标
	void getR(Mat& mask, ArmorRect& present, ArmorRect &former);
	bool judgeRPosition(RotatedRect& R, ArmorRect& present, ArmorRect &former);
	Point2f getCoutourCenter(const std::vector<Point>& coutour);

	bool isValidCenterFansContour(const std::vector<Point>& countour);
	bool isValidCenterArmorContour(const std::vector<Point>& contour);
	void newSpeedMeasure(ArmorRect& present, ArmorRect& former)
	{
		// 这个算法需要计算半径，半径也很重要
		double h = (double)(present.center.y - center_R.center.y);
		double w = (double)(present.center.x - center_R.center.x);
		//double r = sqrt(h * h + w * w);
		double r = present.to_center.getNorm();
		double delta_h = (double)(present.center.y - former.center.y);
		double delta_w = (double)(present.center.x - former.center.x);
		double radio1 = delta_h / sqrt(r * r - h * h) / PI * 180;
		double radio2 =  - delta_w / sqrt(r * r - w * w) / PI * 180;
		double eps = 0.1;
		int iteratorTime = 10;
		if (radio1 < 5 && radio2 < 5) // 这个10emmm,主要是为了消除异常值，这里需要改动
		{
			if (h > 2 * w)
			{
				NEWSPEED = radio1;
			}
			else if (w > 2 * h)
			{
				NEWSPEED = radio2;
			}
			else
				//NEWSPEED = (delta_w / h + delta_h / w) / 2 / PI * 180;
				NEWSPEED = (radio1 + radio2) / 2;

			std::cout << "newSpeed: " << NEWSPEED << std::endl;
		}
	}

	void updateSpeed(ArmorRect& present, ArmorRect& former, Mat& frame)
	{
		// y = 0.785*sin(1.884*t)+1.305
		double Speed = present.speed;// 如果没有找到装甲板，则这个速度将是之间的装甲板的速度(已更新的速度)
		if (Speed > 0.1 && is_find)//设为0.1真的好吗, 要在找到了装甲板的时候进行速度更新
		{
			++valid_speed_num;
			rune_measure_speed.push(Speed);
			Speed = kf.correct_value(Speed);
			//energy.setSpeed(Speed);
			present.setSpeed(Speed); // 更新速度
			if (rune_measure_speed.size() > speed_max_num)
				rune_measure_speed.pop();
			meanOfSpeed();

			if(!is_big)
				predict(present, frame);				// 小符预测
			else
			{
                if (valid_speed_num < 50)
					predict(present, frame);
				else
					bigPredict4(present, former, frame);	// 大符预测 --- 牛逼方案
			}
			putText(frame, "UPDATE: " + std::to_string(Speed), Point(100, 100), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);
		}
	}

	void bigPredict4(ArmorRect& present, ArmorRect& former, Mat& frame)
	{
		if (!is_find) return;
		RMvector vec = present.to_center;
		double res = bigRunePredict(getTime);
		double theta = res * PI / 180;
		double newx, newy;
		if (SHUN == 1)
		{
			theta = -theta;
			newx = center_R.center.x - (vec.x * cos(theta) + vec.y * sin(theta));
			newy = center_R.center.y - (-vec.x * sin(theta) + vec.y * cos(theta));
			circle(frame, Point(newx, newy), 2, Scalar(0, 255, 0), 2);
			pre_x = newx; pre_y = newy;
		}
		else if (SHUN == 2)
		{
			newx = center_R.center.x - (vec.x * cos(theta) + vec.y * sin(theta));
			newy = center_R.center.y - (-vec.x * sin(theta) + vec.y * cos(theta));
			circle(frame, Point(newx, newy), 2, Scalar(0, 255, 0), 2);
			pre_x = newx; pre_y = newy;
		}
	}
	void meanOfSpeed()
	{
		//double sum = 0;
		//for (auto &v:rune_measure_speed)
		//{
		//	sum += v;
		//}
		//meanSpeed = sum / rune_measure_speed.size();
		//std::cout << meanSpeed << std::endl;
		//int len = rune_measure_speed.size();
		int len = valid_speed_num;
		meanSpeed = ((len - 1) * meanSpeed + rune_measure_speed.back())/len;
		//std::cout << meanSpeed << std::endl;
	}

	void drawCircle(Mat& frame, ArmorRect& present)
	{
		circle(frame, center_R.center, present.to_center.getNorm(), Scalar(240, 240, 100), 2);
	}

	double calculateW(RMarray& arr)
	{
		int points_num = arr.size();
		RMarray cos_data(points_num);
		RMarray sin_data(points_num);
		double measureW = 0;	// 测出来正确的W
        double baseW = 0.03; // 大概结果在0.0387左右
		double gapW = 0.0001;
		double W;
        double precise_num = 250;
		std::vector<double> w_res(precise_num);
		std::vector<double> w_res2(precise_num);
		std::vector<double> result(precise_num);
		for (int i = 0; i < precise_num; i++)
		{
			W = baseW + i * gapW;
			for (int j = 0; j < points_num; j++)
			{
				double value = cos(W * (j + valid_speed_num - arr.size()));
				double value2 = sin(W * (j + valid_speed_num - arr.size()));
				cos_data[j] = value;
				sin_data[j] = value;
			}
			w_res[i] = arr.dot(cos_data);
			w_res2[i] = arr.dot(sin_data);
			//cos_data.clear();
		}
		for (int i = 0; i < precise_num; i++)
		{
			result[i] = sqrt(w_res[i] * w_res[i] + w_res2[i] * w_res2[i]);
		}

		// 找绝对值最大的那个下标
		int max_idx = 0;
		int secondMax = 0;
		double max_val = 0;
		for (int i = 0; i < precise_num; i++)
		{
			if (fabs(result[i]) > max_val)
			{
				max_val = fabs(result[i]);
				secondMax = max_idx;
				max_idx = i;
			}
		}
		measureW = baseW + max_idx * gapW;
		//if (fabs(measureW - TW) > 0.001)
		//{
		//	if (fabs(baseW + secondMax * gapW - measureW) < 0.001)
		//	{
		//		measureW = baseW + secondMax * gapW;
		//	}
		//}

		return measureW;
	}
	
	int calculateBias(RMarray& arr, double measureW)
	{
		int points_num = arr.size();
		int max_idx = 0;
		double max_val = 0;
		/*******生成正弦序列****/
		RMarray sin_data(arr.size());
		for (int i = 0; i < points_num; i++)
		{
			double value = sin(measureW * (i + (valid_speed_num - arr.size())));
			sin_data[i] = value;
		}
		/********计算相位差*******/
		std::vector<double> phi_res(2 * points_num - 1);
		// 互相关函数算法
		for (int i = -points_num + 1; i < points_num; i++)
		{
			double temp = 0;
			if (i >= 0)
			{
				for (int j = 0; j < points_num - i; j++)
				{
					temp = temp + sin_data[j] * arr[i + j];
				}
			}
			else
			{
				for (int j = 0; j < points_num + i; j++)
				{
					temp = temp + sin_data[j - i] * arr[j];
				}
			}
			phi_res[i + points_num - 1] = temp / points_num;
		}

		//int secondMax = 0;
		for (int i = 0; i < 2 * points_num - 1; i++)
		{
			if (phi_res[i] > max_val)
			{
				max_val = phi_res[i];
				//secondMax = max_idx;
				max_idx = i;
			}
		}
		int res = max_idx - points_num + 1;
		return res;
	}

	double calculateA(RMarray &arr, double measureW)
	{
		//// 这个频率的计算是一个大概值，精度并不是特别高
		int points_num = arr.size();
		/*******生成正弦序列****/
		RMarray sin_data(arr.size());
		for (int i = 0; i < points_num; i++)
		{
			double value = sin(measureW * (i + (valid_speed_num - arr.size())));
			sin_data[i] = value;
		}
		/*******生成余弦序列****/
		RMarray cos_data(arr.size());
		for (int i = 0; i < points_num; i++)
		{
			double value = cos(measureW * (i + (valid_speed_num - arr.size()))) ;
			cos_data[i] = value;
		}
		double imag = arr.dot(sin_data);
		double real = arr.dot(cos_data);
		return sqrt(imag * imag + real * real) * 2 / (double)points_num;

	}

	void bigRunePraPredict()
	{
		RMarray arr(rune_measure_speed); // 应该有一个最大的长度阈值
		int points_num = arr.size();
		/********数据预处理*******/
		for (int i = 0; i < points_num; i++)
			arr[i] -= TC;
		for (int i = 0; i < points_num; i++)
		{
			double tempSum = 0;
			int cnt = 0;
			for (int j = -7; j <= 7; j++)
			{
				if (i + j >= 0 && i + j < points_num)
				{
					tempSum += arr[i + j];
					++cnt;
				}
			}
			if(cnt)
				arr[i] = tempSum / cnt;
		}

		/********计算频率*******/
        //double measureW = calculateW(arr);
        double measureW = 0.0375;
		/********计算时延*******/
        int max_idx = calculateBias(arr, measureW);
		/********计算幅度*******/
        //double AA = calculateA(arr, measureW);

        TA = 0.65;//0.785
        TC = 1.305;//1.305
        TW = measureW;//0.0384 // 0.01// 0.0000
		BIAS = max_idx;	
	}

	double pow(int d, int t)
	{
		double res = 1;
		for (int i = 0; i < t; i++)
			res *= d;
		return res;
	}

	double robeger(int idx, int frame_num)
	{
		// 龙贝格积分法
		// (TA * sin(TW * (idx + i - BIAS)) + TC)
		const int row = 50;
		double value[row][4];
		double eps = 0.00001;
		double res = 0;
		value[0][0] = 0.5 * frame_num * ((TA * sin(TW * (idx - BIAS)) + TC) + (TA * sin(TW * (idx + frame_num - BIAS)) + TC));
		for (int i = 1; i < row; i++)
		{
			double h = (frame_num) / (double)(pow(2, (i - 1)));
			double rr = 0;
			double r = idx + h / 2;
			while (r <= idx + frame_num - h / 2)
			{
				rr += (TA * sin(TW * (r - BIAS)) + TC);
				r += h;
			}
			value[i][0] = 0.5 * value[i - 1][0] + h / 2 * rr;

			int col = (i > 3) ? 3 : i;
			for (int j = 1; j <= col; j++)
			{
				value[i][j] = ((double)pow(4,j) / ((double)pow(4 , j) - 1) * value[i][j - 1]) - (1.0 / ((double)pow(4, j) - 1) * value[i - 1] [j - 1]);
			}
			if (i > 3)
			{
				res = value[i][3];
			}
			if (i >  3 && fabs(value[i][3] - value[i - 1][3]) < eps)
				break;
		}
		return res;
	}

	double bigRunePredict(double frame_num)
	{
        if (valid_speed_num == 50 || valid_speed_num % 50 == 0)
			bigRunePraPredict();
		int idx = valid_speed_num;
		double preSpeed = TA * sin(TW * (idx - BIAS)) + TC;
		double sum = robeger(idx, frame_num);
		double preAngle = sum;
////        std::cout << "preAngle: " << preAngle << std::endl;
//        std::cout<<"--DAFU---------"<<std::endl;
//        std::cout << "TW: " << TW << std::endl;
//        std::cout << "BIAS: " << BIAS<< std::endl;
//        std::cout << "preSpeed: " << preSpeed << std::endl;
        std::cout << "meanSpeed: " << meanSpeed << std::endl;
        std::cout << "A" << TA << std::endl;
		return preAngle;
	}
};
