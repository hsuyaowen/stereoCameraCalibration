#pragma once
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <time.h>
#include <iomanip>  // for set word width when cout: setw(n)

#define CORNER_NUMBER_HORIZONTAL      6
#define CORNER_NUMBER_VERTICAL        4

#if 1
#define VIDEO_INPUT
#else
#define IMAGE_INPUT
#endif // 1

#ifdef VIDEO_INPUT
#define SOURCEfromFILE           0
#define SOURCEfromDEVICE         1
#define GET_FRAME_INTERVAL      14
#define NEED_FRAME_COUNTS       14
#endif // VIDEO_INPUT

#if 1
#define SINGLE_CAL_DETAIL
#else
#define SINGLE_CAL_REGULAR
#endif

using namespace cv;
using namespace std;

class StereoCameraCalibrator
{
private:
	Size calibration_board_size;
	Size calibration_resolution;
	vector<vector<Point2f>> left_image_points, right_image_points;
	vector<vector<Point3f>> object_points;
	Mat left_cameraMatrix, right_cameraMatrix;
	Mat left_distorMatrix, right_distorMatrix;
	vector<Mat> left_rvecs, right_rvecs;
	vector<Mat> left_tvecs, right_tvecs;
	Mat R, T, E, F;
	Mat R1, R2, P1, P2, Q;

public:
	void setCalibrationBoardSize(const Size &choose_board_size);
	bool addCheckerboardPoints();
	void addPoints(const vector<Point2f> &left_image_corners, const vector<Point2f> &right_image_corners, const vector<Point3f> &object_corners);
	void singleCalibrate();
	void printSingleCalibrateResult();
	void doStereoCalibration();
	void saveCalibrateResult();
	void undistorImage(const Mat &input_image_L, const Mat &input_image_R, Mat &output_image_L, Mat &output_image_R);
};