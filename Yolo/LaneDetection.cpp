#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

#define FILTER_SIZE 5

Mat img_bgr, img_hsv, img_binary, img_gray, img_edge, img_contours, img_roi, img_hough, img_line, img_annotated, img_result;
Point2f leftP[2], rightP[2], vanishP;
Point p0 = Point(0, 0);


//hsv 트랙바에서 사용되는 변수 
int LowH = 0; //0;//0
int HighH = 144; //52;//17
int LowS = 118; //0;//150
int HighS = 255; //67;//255
int LowV = 118; //88
int HighV = 255; //255
				 //Canny edge 트랙바 변수
int lowThreshold = 50;
int highThreshold = 150;

double g_Sum_x;
double g_Buf_x[FILTER_SIZE];
int g_Cnt_x;
double g_Filtered_x;

double g_Sum_y;
double g_Buf_y[FILTER_SIZE];
int g_Cnt_y;
double g_Filtered_y;

double movingAFilter_x(int input) {

	//int sum = 0 ;
	g_Sum_x -= g_Buf_x[g_Cnt_x];
	g_Buf_x[g_Cnt_x++] = input;
	g_Sum_x += input;

	if (g_Cnt_x == FILTER_SIZE) g_Cnt_x = 0;

	return (double)(g_Sum_x) / FILTER_SIZE;

}

double movingAFilter_y(int input) {

	//int sum = 0 ;
	g_Sum_y -= g_Buf_y[g_Cnt_y];
	g_Buf_y[g_Cnt_y++] = input;
	g_Sum_y += input;

	if (g_Cnt_y == FILTER_SIZE) g_Cnt_y = 0;

	return (double)(g_Sum_y) / FILTER_SIZE;
}


void filter_color()
{	//특정색만 추출하는 함수
	cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV); //hsv영상으로 변환
	inRange(img_hsv, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), img_binary); //특정색 검출

																						 //morphological opening 작은 점들을 제거 
	erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing 영역의 구멍 메우기 
	dilate(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
}

void line_detect(Mat &img_line, vector<Vec2f> lines)
{	//대표차선 추출하는 함수

	img_line = img_bgr.clone();

	vector<Vec2f> right_lines;
	vector<Vec2f> left_lines;

	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec2f l = lines[i];		// 여기에 무슨 변수가 저장되는거지?

		double rho = l[0];
		double theta = l[1];

		if (theta < CV_PI * 2 / 5)	//72도
		{
			right_lines.push_back(l);	//전성호 소스
			//left_lines.push_back(l); //왼쪽 차선 추출
		}
		else if (theta > CV_PI * 3 / 5)	//108도		-> 어쨋든 나중에 바꿔야 하는 수치. 이유 : 카메라의 위치에 따라 선의 각도가 다르게보임
		{
			left_lines.push_back(l); //전성호 소스
			//right_lines.push_back(l); //오른쪽 차선 추출
		}

		float resultLine[2][2];

		for (int i = 0; i < left_lines.size(); i++)
		{
			// Theta가 가장 큰 선 1개만 검출
			resultLine[0][0] = left_lines[0][0];
			resultLine[0][1] = left_lines[0][1];

			for (size_t i = 1; i < left_lines.size(); i++)
			{
				if (left_lines[i][1] >= resultLine[0][1])	//바꿈
				{
					resultLine[0][0] = left_lines[i][0];
					resultLine[0][1] = left_lines[i][1];
				}
			}

			double rho = resultLine[0][0];
			double theta = resultLine[0][1];

			//선그리기 할려면 좌표값 계산해야함
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;
			double x1 = (cvRound(x0 + (img_edge.rows + img_edge.cols) * (-b)));
			double y1 = (cvRound(y0 + (img_edge.rows + img_edge.cols) * (a)));// +img_edge.rows / 2);
			double x2 = (cvRound(x0 - (img_edge.rows + img_edge.cols) * (-b)));
			double y2 = (cvRound(y0 - (img_edge.rows + img_edge.cols) * (a)));// +img_edge.rows / 2);

			leftP[0].x = x1;
			leftP[0].y = y1;
			leftP[1].x = x2;
			leftP[1].y = y2;

			line(img_line, Point(x1, y1), Point(x2, y2), Scalar(255, 255, 0), 4, CV_8UC3); //왼쪽 차선 그리기
		}

		for (int i = 0; i < right_lines.size(); i++)
		{
			// Theta가 가장 작은 선 1개만 검출
			resultLine[1][0] = right_lines[0][0];
			resultLine[1][1] = right_lines[0][1];

			for (size_t i = 1; i < right_lines.size(); i++)
			{
				if (right_lines[i][1] < resultLine[1][1])
				{
					resultLine[1][0] = right_lines[i][0];
					resultLine[1][1] = right_lines[i][1];
				}
			}
			double rho = resultLine[1][0];
			double theta = resultLine[1][1];

			//선그리기 할려면 좌표값 계산해야함
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;
			double x1 = (cvRound(x0 + (img_edge.rows + img_edge.cols) * (-b)));
			double y1 = (cvRound(y0 + (img_edge.rows + img_edge.cols) * (a)));// +img_edge.rows / 2);
			double x2 = (cvRound(x0 - (img_edge.rows + img_edge.cols) * (-b)));
			double y2 = (cvRound(y0 - (img_edge.rows + img_edge.cols) * (a)));// +img_edge.rows / 2);

			rightP[0].x = x1;
			rightP[0].y = y1;
			rightP[1].x = x2;
			rightP[1].y = y2;

			line(img_line, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 255), 4, CV_8UC3); //오른쪽 차선 그리기
		}
	}
}

int main()
{
	

	VideoCapture videoCapture(1); //웹캠 불러오기
	printf("asdasd");
	//VideoCapture videoCapture("yellow_test3.mp4"); //차선인식동영상 불러오기
												   

	if (!videoCapture.isOpened())
	{
		cout << "동영상을 열수 없습니다. \n" << endl;

		char a;
		cin >> a;

		return -1;
	}

	/*// hsv 트랙바
	namedWindow("특정색 추출", WINDOW_AUTOSIZE);
	createTrackbar("LowH", "특정색 추출", &LowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "특정색 추출", &HighH, 179);
	createTrackbar("LowS", "특정색 추출", &LowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "특정색 추출", &HighS, 255);
	createTrackbar("LowV", "특정색 추출", &LowV, 255); //Value (0 - 255)
	createTrackbar("HighV", "특정색 추출", &HighV, 255);*/

	while (1)
	{
		videoCapture.read(img_bgr); //원본영상 불러옴
		printf("1");
		if (img_bgr.empty()) break;

		filter_color(); //특정색 추출
						//cvtColor(img_bgr, img_gray, COLOR_BGR2GRAY); //영상 흑백으로 변환
		printf("2");
		GaussianBlur(img_binary, img_binary, Size(5, 5), 0, 0); //가우시안 블러(Size로 블러세기 조절,숫자가 클수록 뭉개짐)
		Canny(img_binary, img_edge, lowThreshold, highThreshold); //캐니엣지 검출
		printf("3");
		//바로 윤곽선 따온다
		Mat img_roi(img_edge, Rect(0, 0, img_edge.cols, img_edge.rows)); //이미지 관심영역 설정
		printf("b");
		vector<Vec2f> lines;
		HoughLines(img_roi, lines, 1, CV_PI / 180, 100, 0, 0, 0, CV_PI);
		printf("a");
		line_detect(img_line, lines); //차선 추출
		

		 //소실점 검출
		float leftLineA = (float)(leftP[1].y - leftP[0].y) / (float)(leftP[1].x - leftP[0].x);
		float leftLineB = leftP[1].y - leftLineA * leftP[1].x;
		float rightLineA = (float)(rightP[1].y - rightP[0].y) / (float)(rightP[1].x - rightP[0].x);
		float rightLineB = rightP[1].y - rightLineA * rightP[1].x;

		// 1차 소실점: 좌우 1차선의 교점
		vanishP.x = (double)((rightLineB - leftLineB) / (leftLineA - rightLineA));
		vanishP.y = (double)(leftLineA * vanishP.x + leftLineB);

		vanishP.x = movingAFilter_x(vanishP.x);
		vanishP.y = movingAFilter_y(vanishP.y);
		printf("%d ,%d", vanishP.x, vanishP.y);
		
		double slope;
		if (vanishP.x - img_line.cols * 0.5 == 0)
			slope = 999.00; // 직진
		else
		{
			if (vanishP.y < 0)
				slope = (float)(img_line.rows - vanishP.y) / (float)(vanishP.x - img_line.cols * 0.5);
			else
				slope = (float)(img_line.rows - vanishP.y) / (float)(vanishP.x - img_line.cols * 0.5);


		}
		printf("기울기 : %f\n", slope);
		if (vanishP.x < 0) {
			continue;
		}
		// 여기서 line 함수 써주기전에 vanishP 라는 점을 이동평균 필터를 통해 값 보정을 해주어야 한다.
		line(img_line, Point(img_line.cols * 0.5, img_line.rows), vanishP, Scalar(0, 255, 0), 4, CV_8UC3);

		//hconcat(img_line, img_roi, img_result); //영상 가로로 붙이기
		//imshow("원본 영상", img_bgr);
		//imshow("이진화 영상", img_binary);
		//imshow("엣지 검출", img_edge);
		imshow("차선검출", img_line);
		imshow("관심영역", img_roi);


		if (waitKey(1) == 27) break; //ESC키 누르면 종료 
		/*
		leftP[0] = Point(0,0);
		leftP[1] = Point(0, 0);
		rightP[0] = Point(0, 0);
		rightP[1] = Point(0, 0);*/

	}

	return 0;

}
