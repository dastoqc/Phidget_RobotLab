#include <iostream>
#include <fstream>
#include <time.h>
#include <windows.h>

#include "PhidgetClass.h"
#include "arucofct.h"

#include "stdafx.h"

using namespace std;

CPhidgetWrapper pw[2];
arucofct AF;

#define WMOT 0	//0,1,2
#define REC 1


/**
* @function main
*/
int main(int argc, char** argv)
{
	char key = 0;
	double target1 = 0.0, target2 = 0.0;

	// Create a CSV file with current date/time.
	time_t t = time(0);   // get time now
	struct tm * now_t = localtime(&t);
	char buffer[80];
	ofstream csvfile;
	if (REC) {
		strftime(buffer, 80, "%Y-%m-%d-%H%M.csv", now_t);
		csvfile.open(buffer);
		if (!csvfile.is_open())
		{
			cout << "Error opening csv file." << endl;
			return -1;
		}
	}

	if (!pw[0].Init(0)) // init 0 for dc motor, 1 for stepper
		return -1; // if init return 0 it fails. we quit.
	pw[0].setGains(1.5, 0.9, 0.08, 0.01); // P, I, D gains (for wrapper current, velocity or position control) and PID deadband
	pw[0].setlimits(100, 8.35, 42.0, 0.0); //MaxVelocity, MinVelocity (for motor command without deadband) and input velocity limits in r/s

	if (WMOT > 1) {	//again for the second motor.
		if (!pw[1].Init(0))
			return -1;
		pw[1].setGains(1.7, 0.0, 0.03, 0.001);
		pw[1].setlimits(100, 7, 42.0, 0.0);
	}

	//Aruco markers tracking
	double dt = 10; //ms
	AF.start_tracker(REC, dt);

	SYSTEMTIME time; GetSystemTime(&time);
	double start = (double)time.wMinute * 60.0 + time.wSecond + (double)time.wMilliseconds / 1000.0; //for timestamps
	double now = (double)time.wMinute * 60.0 + time.wSecond + (double)time.wMilliseconds / 1000.0; //for timestamps
	/*
	* LOOP
	*/
	do {
		GetSystemTime(&time);
		now = (double)time.wMinute * 60.0 + time.wSecond + (double)time.wMilliseconds / 1000.0;
		target1 = 42 * sin((now - start) * 0.4);// play a sinus
		cout << "Velocity :" << target1 << "% (" << (now - start) << "s)" << endl;

		// Send directly velocity to motors (% of maximum voltage)
		/*if (WMOT > 0) {
			pw[0].setvel(target1,1);	//set to 1 to use deadband remaping.
			if (WMOT > 1)	pw[1].setvel(target1,1);
		}*/

		// Velocity control with encoder feedback.
		if (WMOT > 0) {
			pw[0].target = target1;	//set the target position
			pw[0].started_vel = true;	//start the PID
		}
		if (WMOT > 1) {
			pw[1].target = target1;
			pw[1].started_vel = true;
		}

		/*
		// Position control with encoder feedback.
		pw[0].target=target1;	//set the target position
		pw[0].started_pos=true;	//start the PID
		if(WMOT>1)	pw[1].target=target2;
		if(WMOT>1)	pw[1].started_pos=true;*/

		/*
		// Current control with current feedback. !!Never tested!!
		pw[0].target=target2;
		pw[0].started_cur=true;
		if(WMOT>1)	pw[1].target=target2;
		if(WMOT>1)	pw[1].started_cur=true;*/

		/*
		// Run a stepper
		__int64 stepcommand = pw[0].rad2steps(target1);
		if (abs(stepcommand)>10000)	stepcommand = sgn(stepcommand) * 10000;
		pw[0].GoToStepper(0, stepcommand);*/


		// Track markers
		AF.getmarkers(REC, dt);

		// Record data.
		if(REC)
			csvfile << now-start << ";" << target1 << ";" << pw[0].curr_pos << "; " << pw[1].curr_pos << "; " << pw[0].vel_raw[9] << "; " << pw[1].vel_raw[9] << "; " << pw[0].vel_fil[9] << "; " << pw[1].vel_fil[9] << "; " << pw[0].curr_accel << "; " << pw[1].curr_accel << "; " << pw[0].cur_fil[9] << "; " << pw[1].cur_fil[9] << "; " << endl;

		key = cv::waitKey(dt);//wait for key to be pressed
	}while (key != 27); //stop program with esc key.

	/// CLOSING
	if (WMOT) {
		pw[0].closeMot();
		if (WMOT>1)	pw[1].closeMot();
	}

	if (REC)	csvfile.close();

	return 0;
}
