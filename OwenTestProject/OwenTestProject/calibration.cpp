#include "calibration.h"

void StereoCameraCalibrator::setCalibrationBoardSize(const Size &choose_board_size)
{
	cout << "set checkerboard inner corner size" << endl;

	calibration_board_size = choose_board_size;
	
	cout << calibration_board_size << endl << endl;
}

bool StereoCameraCalibrator::addCheckerboardPoints()
{
#ifdef VIDEO_INPUT
	int video_source = SOURCEfromFILE;               // set video source
#endif
	VideoCapture video;                              // new a video object
	Mat video_frame;                                 // video frame container
	unsigned int successGetFrameCount = 0;
	unsigned long frameNumber = 0;
	vector<Point2f> temp_image_corners_L, temp_image_corners_R;
	vector<Point3f> temp_object_corners;

	cout << "collecte inner corner pattern..." << endl;

	// initialize object points
	for (int i = 0; i < calibration_board_size.height; i++)
		for (int j = 0; j < calibration_board_size.width; j++)
			temp_object_corners.push_back(Point3f((float)i, (float)j, 0.0f));

	if (video_source == SOURCEfromFILE)
	{
		video.open("input_video.avi");
		if (!video.isOpened())
		{
			cout << "failed to open video file" << endl;
			return false;
		}
	}
	else if(video_source == SOURCEfromDEVICE)
	{
		video.open(0);
		if (!video.isOpened())
		{
			cout << "failed to open camera device" << endl;
			return false;
		}
	}

	calibration_resolution = Size((int)video.get(CV_CAP_PROP_FRAME_WIDTH), (int)video.get(CV_CAP_PROP_FRAME_HEIGHT));
	while (successGetFrameCount < NEED_FRAME_COUNTS)
	{
		video >> video_frame;
		if (frameNumber++ % GET_FRAME_INTERVAL == 0)
		{
			bool success_L = false, success_R = false;
			Mat gray_video_frame_L = video_frame.clone();
			Mat gray_video_frame_R = video_frame.clone();
			cvtColor(video_frame, gray_video_frame_L, CV_RGB2GRAY);
			cvtColor(video_frame, gray_video_frame_R, CV_RGB2GRAY);
			if (gray_video_frame_L.empty() || gray_video_frame_L.empty())
			{
				cout << "fail when getting gray frame" << endl;
				return false;
			}

			success_L = findChessboardCorners(gray_video_frame_L, calibration_board_size, temp_image_corners_L, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
			success_R = findChessboardCorners(gray_video_frame_R, calibration_board_size, temp_image_corners_R, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

			if (success_L && success_R)
			{
				++successGetFrameCount;
				TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
				cornerSubPix(gray_video_frame_L, temp_image_corners_L, Size(5, 5), Size(-1, -1), param);
				cornerSubPix(gray_video_frame_R, temp_image_corners_R, Size(5, 5), Size(-1, -1), param);
				if ((temp_image_corners_L.size() == calibration_board_size.area()) && (temp_image_corners_L.size() == calibration_board_size.area()))
					addPoints(temp_image_corners_L, temp_image_corners_R, temp_object_corners);
				drawChessboardCorners(gray_video_frame_L, calibration_board_size, temp_image_corners_L, success_L);
				drawChessboardCorners(gray_video_frame_R, calibration_board_size, temp_image_corners_R, success_R);
			}
			else
			{
				cout << "failed to find all corners" << endl;
			}
			/* show each pattern in gray
			//namedWindow("L gray frame", CV_WINDOW_NORMAL);   resizeWindow("L gray frame", 340, 200);   moveWindow("L gray frame", 0, 0);
			//namedWindow("R gray frame", CV_WINDOW_NORMAL);   resizeWindow("R gray frame", 340, 200);   moveWindow("R gray frame", 340, 0);
			//imshow("L gray frame", gray_video_frame_L);
			//imshow("R gray frame", gray_video_frame_R);
			//waitKey(20);
			*/
		}
		else
			continue;
	}
	destroyAllWindows();
	cout << endl;
	return true;
}


void StereoCameraCalibrator::addPoints(const vector<Point2f> &left_image_corners, const vector<Point2f> &right_image_corners, const vector<Point3f> &object_corners)
{
	static unsigned int count = 0;
	left_image_points.push_back(left_image_corners);
	right_image_points.push_back(right_image_corners);
	object_points.push_back(object_corners);
	cout << "have collected " << setw(2) << ++count << " pattern" << endl;
}


void StereoCameraCalibrator::singleCalibrate()
{
	cout << "now calibration..." << endl;

#if defined SINGLE_CAL_DETAIL
	vector<double> L_stdDeviationsIntrinsics, L_stdDeviationsExtrinsics, R_stdDeviationsIntrinsics, R_stdDeviationsExtrinsics;
	vector<double> L_perViewErrors, R_perViewErrors;
	double L_mean_RMS_error = 0., R_mean_RMS_error = 0.;

	calibrateCamera(object_points, left_image_points, calibration_resolution, left_cameraMatrix, left_distorMatrix, left_rvecs, left_tvecs, L_stdDeviationsIntrinsics, L_stdDeviationsExtrinsics, L_perViewErrors);
	calibrateCamera(object_points, right_image_points, calibration_resolution, right_cameraMatrix, right_distorMatrix, right_rvecs, right_tvecs, R_stdDeviationsIntrinsics, R_stdDeviationsExtrinsics, R_perViewErrors);
	
	// show view RMS error
	for (int i = 0; i < L_perViewErrors.size(); i++)
	{
		cout << "L RMS re-projection error of" << setw(3) << i + 1 << "th pattern = " << L_perViewErrors[i] << endl;
		cout << "R RMS re-projection error of" << setw(3) << i + 1 << "th pattern = " << R_perViewErrors[i] << endl;
		L_mean_RMS_error += L_perViewErrors[i];
		R_mean_RMS_error += R_perViewErrors[i];
	}
	cout << "mean L side RMS re-projection error = " << L_mean_RMS_error/L_perViewErrors.size() << endl;
	cout << "mean R side RMS re-projection error = " << R_mean_RMS_error/R_perViewErrors.size() << endl;
	// show view RMS error
#elif defined SINGLE_CAL_REGULAR
	cout << "mean L side RMS re-projection error = " << calibrateCamera(object_points, left_image_points, calibration_resolution, left_cameraMatrix, left_distorMatrix, left_rvecs, left_tvecs);
	cout << "mean R side RMS re-projection error = " << calibrateCamera(object_points, right_image_points, calibration_resolution, right_cameraMatrix, right_distorMatrix, right_rvecs, right_tvecs);
#endif // SINGLE_CAL_DETAIL
	
	cout << endl;
}


void StereoCameraCalibrator::printSingleCalibrateResult()
{
	cout << "calibration finish." << endl;
	cout << "# left_cameraMatrix" << endl << left_cameraMatrix << endl
		<< "# right_cameraMatrix" << endl << right_cameraMatrix << endl;
	cout << "# left_distorMatrix" << endl << left_distorMatrix << endl
		<< "# right_distorMatrix" << endl << right_distorMatrix << endl;

	/* print out the translation vector of each pattern calibration result 
	for (int i = 0; i < left_tvecs.size(); ++i)
	{
		cout << "# left_tvecs[" << i << "]" << endl << left_tvecs[i] << endl;
		cout << "# right_tvecs[" << i << "]" << endl << right_tvecs[i] << endl;
	}*/

	cout << endl;
}


void StereoCameraCalibrator::doStereoCalibration()
{
	cout << "Now doing stereo calibration..." << endl;

	stereoCalibrate(object_points, left_image_points, right_image_points, left_cameraMatrix, left_distorMatrix, right_cameraMatrix, right_distorMatrix, \
		calibration_resolution, R, T, E, F, CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);
	stereoRectify(left_cameraMatrix, left_distorMatrix, right_cameraMatrix, right_distorMatrix, calibration_resolution, R, T, R1, R2, P1, P2, Q);

	cout << "stereo calibration finish." << endl << endl;
}


void StereoCameraCalibrator::saveCalibrateResult()
{
	FileStorage fs_L("singleCalibrationResult_Left.yml", FileStorage::WRITE);
	FileStorage fs_R("singleCalibrationResult_Right.yml", FileStorage::WRITE);
	FileStorage fs_stereo("stereoCalibrationResult.yml", FileStorage::WRITE);
	time_t now; time(&now);
	struct tm tmTmp;
	char stTmp[30];
	localtime_s(&tmTmp, &now);
	asctime_s(stTmp, &tmTmp);
	fs_L << "calibrationDate" << stTmp;
	fs_R << "calibrationDate" << stTmp;
	fs_L << "calibration_resolution" << calibration_resolution;
	fs_R << "calibration_resolution" << calibration_resolution;
	fs_L << "Left_intrisicMatrix" << left_cameraMatrix;
	fs_R << "Right_intrinsicMatrix" << right_cameraMatrix;
	fs_L << "Left_distortionMatrix" << left_distorMatrix;
	fs_R << "Right_distortionMatrix" << right_distorMatrix;
	fs_stereo << "R" << R;
	fs_stereo << "T" << T;
	fs_stereo << "E" << E;
	fs_stereo << "F" << F;
	fs_stereo << "R1" << R1;
	fs_stereo << "R2" << R2;
	fs_stereo << "P1" << P1;
	fs_stereo << "P2" << P2;
	fs_stereo << "Q" << Q;
	fs_L.release();
	fs_R.release();
	fs_stereo.release();

	cout << "finish to save calibration result" << endl
		<< "please refer CalibrationResult_Left.yml and CalibrationResult_Right.yml" << endl;
}


void StereoCameraCalibrator::undistorImage(const Mat &input_image_L, const Mat &input_image_R, Mat &output_image_L, Mat &output_image_R)
{
	Mat map1_L, map2_L, map1_R, map2_R;

	initUndistortRectifyMap(left_cameraMatrix, left_distorMatrix, Mat(), Mat(), calibration_resolution, CV_32F, map1_L, map2_L);
	initUndistortRectifyMap(right_cameraMatrix, right_distorMatrix, Mat(), Mat(), calibration_resolution, CV_32F, map1_R, map2_R);
	remap(input_image_L, output_image_L, map1_L, map2_L, INTER_LINEAR);
	remap(input_image_R, output_image_R, map1_R, map2_R, INTER_LINEAR);

	namedWindow("L distorton image", CV_WINDOW_NORMAL);   resizeWindow("L distorton image", 340, 200);   moveWindow("L distorton image", 0, 0);
	namedWindow("R distorton image", CV_WINDOW_NORMAL);   resizeWindow("R distorton image", 340, 200);   moveWindow("R distorton image", 340, 0);
	imshow("L distorton image", input_image_L);
	imshow("R distorton image", input_image_R);

	namedWindow("L undistorton image", CV_WINDOW_NORMAL);   resizeWindow("L undistorton image", 340, 200);   moveWindow("L undistorton image", 0, 200);
	namedWindow("R undistorton image", CV_WINDOW_NORMAL);   resizeWindow("R undistorton image", 340, 200);   moveWindow("R undistorton image", 340, 200);
	imshow("L undistorton image", output_image_L);
	imshow("R undistorton image", output_image_R);

	cout << "undistortion L/R sample images finish." << endl << endl;
	waitKey();
	destroyAllWindows();
}