#ifndef ARUCO_FCT_H
#define ARUCO_FCT_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"
#include "aruco125/cvdrawingutils.h"

using namespace cv;
using namespace aruco;

class arucofct
{
private:
	VideoWriter outputVideo;
	MarkerDetector MDetector;
	VideoCapture TheVideoCapturer;
	vector<Marker> TheMarkers;
	Mat TheInputImage, TheInputImageCopy;
	CameraParameters TheCameraParameters;

	KalmanFilter KF;

public:
	int start_tracker(int REC, double dt);
	int getmarkers(int REC, double dt);
};

#endif