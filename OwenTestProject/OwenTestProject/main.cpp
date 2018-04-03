#include "calibration.h"

int main()
{
	StereoCameraCalibrator xrCameraCalibrator;

	xrCameraCalibrator.setCalibrationBoardSize(Size(CORNER_NUMBER_HORIZONTAL, CORNER_NUMBER_VERTICAL));

	if (xrCameraCalibrator.addCheckerboardPoints() == false)
	{
		waitKey();
		return -1;
	}

	xrCameraCalibrator.singleCalibrate();

	xrCameraCalibrator.printSingleCalibrateResult();

	xrCameraCalibrator.doStereoCalibration();  // Apr03 add

	xrCameraCalibrator.saveCalibrateResult();

	Mat distor_image_L, distor_image_R;
	Mat undistor_image_L, undistor_image_R;
	distor_image_L = imread("sample_L.jpg");
	distor_image_R = imread("sample_R.jpg");
	xrCameraCalibrator.undistorImage(distor_image_L, distor_image_R, undistor_image_L, undistor_image_R);

	return 0;
}