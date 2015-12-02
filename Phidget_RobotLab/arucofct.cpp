#include <iostream>
#include <fstream>
#include <time.h>
#include <windows.h>

#include "arucofct.h"

/// START TRACKER
int arucofct::start_tracker(int REC, double dt) {
	char buffer[80];

	pair<double, double> AvrgTime(0, 0);//determines the average time required for detection
	char key = 0; Mat rotation1Init, rotation2Init; int leftT = 0;
	try {
		//read from camera or from  file
		int vIdx = 0;
		cout << "Opening camera index " << vIdx << endl;
		TheVideoCapturer.open(vIdx);

		//check video is open
		if (!TheVideoCapturer.isOpened()) {
			cerr << "Could not open video" << endl;
			return -1;
		}

		//read first image to get the dimensions		
		//TheVideoCapturer.set(CV_CAP_PROP_CONVERT_RGB, 1);
		//TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
		//TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
		TheVideoCapturer >> TheInputImage;
		//read camera parameters
		TheCameraParameters.readFromXMLFile("camera.yml");
		TheCameraParameters.resize(TheInputImage.size());
		//Resize image?
		MDetector.pyrDown(1);
		//Create debug gui
		cv::namedWindow("Tracking", 1);
		MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
		if (REC) {
			time_t t = time(0);   // get time now
			struct tm * now = localtime(&t);
			strftime(buffer, 80, "%Y-%m-%d-%H%M.avi", now);
			outputVideo.open(buffer, CV_FOURCC('I', 'Y', 'U', 'V'), 25, TheInputImage.size(), true);
			if (!outputVideo.isOpened())
			{
				cout << "Could not open the output video for write. " << endl;
				return -1;
			}
		}
	}
	catch (std::exception &ex) { cout << "Exception :" << ex.what() << endl; }

	KF.init(3, 1);
	// intialization of KF for kinematic system
	KF.transitionMatrix = *(Mat_<float>(3, 3) << 1, dt, dt / 2, 0, 1, dt, 0, 0, 1);
	cout << KF.measurementMatrix.size() << KF.measurementNoiseCov.size() << endl;
	// we measure position and accel.
	KF.measurementMatrix = *(Mat_<float>(1, 3) << 1, 0, 0);
	//discrete prediction noise model
	KF.processNoiseCov = *(Mat_<float>(3, 3) << pow(dt, 5) / 20, pow(dt, 4) / 8, pow(dt, 3) / 6, pow(dt, 4) / 8, pow(dt, 3) / 3, pow(dt, 2) / 2, pow(dt, 3) / 6, pow(dt, 2) / 2, dt);
	// 
	KF.measurementNoiseCov = *(Mat_<float>(1, 1) << 0.01);
	setIdentity(KF.errorCovPost, Scalar::all(5));

	return 1;
}


int arucofct::getmarkers(int REC, double dt) {
	Mat measurement = *(Mat_<float>(1, 1) << 0);

	TheVideoCapturer.retrieve(TheInputImage);//get image
											 //Detection of markers in the image passed
	MDetector.detect(TheInputImage, TheMarkers, TheCameraParameters, 0.046); //markers size 46mm
			 //print marker info and draw the markers in image
	TheInputImage.copyTo(TheInputImageCopy);

	for (unsigned int i = 0; i < TheMarkers.size(); i++) {
		cout << "ID: " << TheMarkers[i].id << endl;
		Mat rotation;
		Rodrigues(TheMarkers[i].Rvec, rotation);
		cout << "Rotation: " << rotation << endl << "Translation: " << TheMarkers[i].Tvec << endl;
		if (TheMarkers[i].id == 599)
			measurement.at<float>(0) = TheMarkers[i].Tvec.at<float>(0);
	}

	//draw a 3d cube in each marker if there is 3d info
	if (TheCameraParameters.isValid())
		for (unsigned int i = 0; i<TheMarkers.size(); i++) {
			CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
			CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
		}
	cv::imshow("Tracking", TheInputImageCopy);
	if (REC)	outputVideo.write(TheInputImageCopy);

	// KF predict, to update the internal statePre variable
	KF.transitionMatrix = *(Mat_<float>(3, 3) << 1, dt, dt / 2, 0, 1, dt, 0, 0, 1);
	KF.processNoiseCov = *(Mat_<float>(3, 3) << pow(dt, 5) / 20, pow(dt, 4) / 8, pow(dt, 3) / 6, pow(dt, 4) / 8, pow(dt, 3) / 3, pow(dt, 2) / 2, pow(dt, 3) / 6, pow(dt, 2) / 2, dt);
	KF.predict();
	// KF update
	Mat estimated1 = KF.correct(measurement);

	return 1;
}