#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <rp_kalman.h>
using namespace cv;//���Ǹ����õı��ϰ�ߣ���������cv::
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
extern int SCREEN_MAX_X;//��Ļ�ֱ���1280x720
extern int SCREEN_MAX_Y;

class RMvector
{
	// ���Ż�
	// ���Ӧ���ǿ��Ըĳ�Point2f�������࣬����뷨�е�Ƿ�ף�xy��ͻ
public:
	double x;							// ��ԭ��Ϊ���ĵ�x����
	double y;							// ��ԭ��Ϊ���ĵ�y����
	double angle;						// �����ĽǶȣ��Ժ���Ϊ��׼
	RMvector()
	{
		x = 0; y = 0;
		angle = 0;
	}
	RMvector(Point2f& start, Point2f& end)
	{
		Point2f vec = end - start;
		x = vec.x; y = vec.y;
		angle = atan(y / x);// ���Ŀ�����Դ
	}
	RMvector(double a)// �ɽǶȼ��㵥λ����
	{
		x = cos(a * PI / 180); y = sin(a * PI / 180);
		angle = a; // ���ܳ���0-360��Χ������ȫ�������Ϊmod
	}
	double getNorm()// ���������ĳ���
	{
		return sqrt(x * x + y * y);
	}
	double dot(RMvector& new_vector)// �����������һ������
	{
		return x * new_vector.x + y * new_vector.y;
	}
	double cross(RMvector& new_vector)// �����������һ��������ע��˳��
	{
		return x* new_vector.y - y * new_vector.x;
	}
};

class ArmorRect
{
	// ������Ըĳ���ת���ε�������
public:
	Point2f center;					// ��ת���ε�����
	double width;					// ��ת���ε�width
	double height;					// ��ת���ε�height
	double long_edge;				// ����
	double short_edge;				// �̱�
	double rotatedrect_angle;		// ��ת����ԭ�еĽǶ�(-90, 0]
	double angle;					// װ�װ�ĽǶȣ�[0, 180);	//���������ǶȾͿ��Լ������ߺͷ��ߵ�������
	double tangent_angle;			// װ�װ����߽Ƕ�
	double area;					// ���
	RMvector to_center;				// ��������
	RMvector tangent;				// ����
	double speed;					// װ�װ�Ľ��ٶȣ�ÿ֡����rad
	bool hitted;					// ��װ�װ��Ƿ񱻵�����־λ // �ⲿ�ӿ�
	RMvector vertex2center[4];
	Point2f vertices[4];
	Point2f fan_center;				// ����Ҷ������
	double eps = 0.35;				// �ٶ���ֵ�� С�������ֵ�ٶ���Ϊ0
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
	void setParam(RotatedRect& rect)// ���ڸ����µ�װ�װ�
	{
		center = rect.center;
		width = rect.size.width;
		height = rect.size.height;
		long_edge = max(width, height);
		short_edge = min(width, height);
		rotatedrect_angle = rect.angle;
		area = width * height;
		setAngel(); // ���ýǶ�
		setTangent();
		speed = 0;
		hitted = 0;
		rect.points(vertices);
	}
	void setSpeed(double w)
	{
		if (w < eps) // ���˳���жϴ������� /// ���������
			speed = 0;
		speed = w;
	}
	void setAngel()
	{// ����
		angle = width > height ? (rotatedrect_angle + 90) :  rotatedrect_angle;
	}
	void setTangent()
	{// ����
		tangent_angle = width < height ? (rotatedrect_angle + 90) : rotatedrect_angle;
	}
	void setToCenter()
	{		
		for (int i = 0; i < 4; i++)
		{
			RMvector vec1(vertices[i], center_R.center);
			vertex2center[i] = vec1; // ����ֱ�Ӹ�ֵ��ֵ����
		}
		RMvector vec(center, center_R.center);// �����������()��������򻯴��룬һ��С�Ż������Ǳ�Ҫ
		to_center = vec;
	}
};

class RMarray
{
private:
	int len;
public:
	std::vector<double> data; // �������д����ģ����ʽ�����Ե�û��Ҫ
	RMarray() 
	{
		data.push_back(0);// ��ֹ��bug
	}
	RMarray(int length) 
	{
		if (length == 0)
		{
			data.push_back(0);// ��ֹ��bug
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
	//	data.clear();// ������0�����������ڴ�
	//	data.push_back(0);
	//	data.swap(R())
	//}
	double& operator[](int i)
	{
		//if (i >= len)
		//{
		//	//cout << "�����������ֵ" << endl;
		//	// Ӧ��throw error
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
		// ����ú�����Ҫ���ã�����һ��string������full,same...
		// ����ֻʵ��same����
		if (arr.size() > len)
		{
			//error
			return 0;
		}
		int arrLen = arr.size();
		std::vector<double> vec;
		double sum = 0;
		for (int i = 0; i < len - arrLen + 1; i++) // �ڻ���ô���
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

// ������Ҫ�Ƕ������������������װ�װ�����������
// һ���ǵ�ǰ��װ�װ壬һ����ǰһ��װ�װ�
class Energy
{
private:
	double TA = 0.785;
	double TC = 1.305;
	double TW = 0.0384;
	int BIAS = -60;
	int THRESH_BR = 56;
	int THRESH_GRAY = 79;
	int MIN_AREA = 200;						// ���п�ʶ�������С�������
	int IS_RED = 0;							// �з�װ�װ���ɫΪ��ɫ��Ϊ1
	int SHUN = 0;							// �ж���˳��ʱ�룬0��ⲻ����1Ϊ˳��2Ϊ��
	double NEWSPEED;						// װ�װ�����ٶ�
	kalman_filter kf;						// ���忨�����˲���
	double former_speed;					// ǰһ֡���ٶ�
	double second_to_frame = 0.03;			// һ֡������
	double bias = 0;						// �Ŷ��Ĵ�С
    int DILATE_EOFF = 9;
	double radius = 3;
//	int pass_frames = 10;
//	int pass_index = 0;
	std::vector<double> initBigSpeed;
	std::queue<double> rune_measure_speed;	// �ٶȶ���
	//std::vector<double> rune_measure_speed;	// �ٶȶ���
    int speed_max_num = 300;				// �ٶȶ���Ԫ�ص���󳤶�
	int begin_bigPredcit_time = 0;
	double meanSpeed;
	int valid_speed_num = 0;
	// �ϴε�����ʱ���ֻ��ɵ�֡�����⣬��������ROI����Բ��
	// ��ʵ�ּ��ٶȴ���Ŷ�Ԥ��
public:
	// ��Ҫ���ظ���ص�ֵ
	float pre_x = 0;						// Ԥ������x����
	float pre_y = 0;						// Ԥ������y����
	int is_find = 0;						// �Ƿ��ҵ�װ�װ�
	// ƫ��ʱ��
	float getTime;
	int time2fire = 0;
	bool is_big = 0;
	Energy() 
	{
		kf.init_kalman_filter(1);				// �������˲��ٶȳ�ʼ��
	}
	// ��ҪһЩ�������캯��
	// ͨ��ÿһ֡ͼ��õ�һ��mask��ͼ��ʶ����Ҫ����
	void videoProcess(Mat& frame, Mat& mask, ArmorRect& present, ArmorRect& former)
	{
		getMask(frame, mask);					// �õ���Ĥ
		/* ------��mask���ҳ�����ȷ��present------- */
		getCoutour(mask, frame, present);		// ��װ�װ������ // �õ��µ�present
		getR(mask, present, former);					// �õ�����R	// Բ����Ҫװ�װ���ȷ�� // ȷ��װ�װ����������
		// ֻ�����ǰ�����������ſ���ȷ��present
		present.setToCenter();
		if (is_find)
		{
			judgeDirect(present, former, frame);	// �ж�˳�� 
			//newSpeedMeasure(present, former);		// �ݲ�ʹ�ã��Ż���(�����ԣ�����ʹ��)
			getSpeed(present, former, frame);		// ������ٶ�
			former = present;						// ����װ�װ���Ϣ
			drawCircle(frame, present);
		}
	}
	void reset()
	{
		//std::cout << pre_y << std::endl;//debug��
		pre_x = 0;
		pre_y = 0;
	}
	// �ⲿ�ӿ�:������ֵ
	void setThresh(int br, int gray) {
		THRESH_BR = br;
		THRESH_GRAY = gray;
	}
	// �ⲿ�ӿ�:�Ƿ�Ϊ��ɫ
	void isRed(bool b)
	{
		IS_RED = b;
	}
	
	void getMask(Mat& frame, Mat& mask);
	// ������ǰ֡��װ�װ��֮ǰ֡��װ�װ������ٶȻ����ǽǶȣ���Ҫ�õ����
	void getSpeed(ArmorRect& present, ArmorRect& former, Mat& frame);
	// Ԥ����Ҫ�����λ�ã���Ҫ����pitch��yaw
	void predict(ArmorRect& present, Mat& frame);
	// �ж���˳ʱ�뻹����ʱ��
	void judgeDirect(ArmorRect& present, ArmorRect& former, Mat& frame);
	// ͨ���������ҵķ����ҵ�װ�װ��λ��
	void getCoutour(Mat& mask, Mat& frame, ArmorRect& armor_center);
	// ��������Ե�����ʶ��R����ʱ��һϵ���ж�
	bool isValidCenterRContour(const std::vector<Point>& center_R_contour);
	// �õ�R���ĵĺ������õ�center_R������
	void getR(Mat& mask, ArmorRect& present, ArmorRect &former);
	bool judgeRPosition(RotatedRect& R, ArmorRect& present, ArmorRect &former);
	Point2f getCoutourCenter(const std::vector<Point>& coutour);

	bool isValidCenterFansContour(const std::vector<Point>& countour);
	bool isValidCenterArmorContour(const std::vector<Point>& contour);
	void newSpeedMeasure(ArmorRect& present, ArmorRect& former)
	{
		// ����㷨��Ҫ����뾶���뾶Ҳ����Ҫ
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
		if (radio1 < 5 && radio2 < 5) // ���10emmm,��Ҫ��Ϊ�������쳣ֵ��������Ҫ�Ķ�
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
		double Speed = present.speed;// ���û���ҵ�װ�װ壬������ٶȽ���֮���װ�װ���ٶ�(�Ѹ��µ��ٶ�)
		if (Speed > 0.1 && is_find)//��Ϊ0.1��ĺ���, Ҫ���ҵ���װ�װ��ʱ������ٶȸ���
		{
			++valid_speed_num;
			rune_measure_speed.push(Speed);
			Speed = kf.correct_value(Speed);
			//energy.setSpeed(Speed);
			present.setSpeed(Speed); // �����ٶ�
			if (rune_measure_speed.size() > speed_max_num)
				rune_measure_speed.pop();
			meanOfSpeed();

			if(!is_big)
				predict(present, frame);				// С��Ԥ��
			else
			{
                if (valid_speed_num < 50)
					predict(present, frame);
				else
					bigPredict4(present, former, frame);	// ���Ԥ�� --- ţ�Ʒ���
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
		double measureW = 0;	// �������ȷ��W
        double baseW = 0.03; // ��Ž����0.0387����
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

		// �Ҿ���ֵ�����Ǹ��±�
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
		/*******������������****/
		RMarray sin_data(arr.size());
		for (int i = 0; i < points_num; i++)
		{
			double value = sin(measureW * (i + (valid_speed_num - arr.size())));
			sin_data[i] = value;
		}
		/********������λ��*******/
		std::vector<double> phi_res(2 * points_num - 1);
		// ����غ����㷨
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
		//// ���Ƶ�ʵļ�����һ�����ֵ�����Ȳ������ر��
		int points_num = arr.size();
		/*******������������****/
		RMarray sin_data(arr.size());
		for (int i = 0; i < points_num; i++)
		{
			double value = sin(measureW * (i + (valid_speed_num - arr.size())));
			sin_data[i] = value;
		}
		/*******������������****/
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
		RMarray arr(rune_measure_speed); // Ӧ����һ�����ĳ�����ֵ
		int points_num = arr.size();
		/********����Ԥ����*******/
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

		/********����Ƶ��*******/
        //double measureW = calculateW(arr);
        double measureW = 0.0375;
		/********����ʱ��*******/
        int max_idx = calculateBias(arr, measureW);
		/********�������*******/
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
		// ��������ַ�
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
