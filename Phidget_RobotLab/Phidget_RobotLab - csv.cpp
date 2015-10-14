#include <iostream>
#include <fstream>
#include <time.h>
#include <windows.h>

#include "PhidgetClass.h"

#include "stdafx.h"

using namespace std;

CPhidgetWrapper pw[2];

#define WMOT 1	//0,1,2
#define REC 1

/**
* @function main
*/
int main(int argc, char** argv)
{
	double target1 = 0.0, target2 = 0.0;
	int leftM = 0;

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
	pw[0].setGains(50, 3, 1, 100, 10); // P, I, D gains then MaxVelocity and MinVelocity

	if (WMOT > 1) {	//again for the second motor.
		if (!pw[1].Init(0))
			return -1;
		pw[1].setGains(50, 3, 1, 100, 10); // P, I, D gains (for wrapper close-loop on position or current) then MaxVelocity and MinVelocity
	}
	if (pw[0].devid == 394129) // hardcoded controller card id for left motor.
		leftM = 0;
	else
		leftM = 1;

	/*
	* LOOP
	*/
	SYSTEMTIME time; GetSystemTime(&time);
	double start = (double)time.wMinute * 60.0 + time.wSecond + (double)time.wMilliseconds / 1000.0; //for timestamps
	double now = (double)time.wMinute * 60.0 + time.wSecond + (double)time.wMilliseconds / 1000.0; //for timestamps
	double target = 0.0;
	while ((now - start) <= 40.0) { //loop for 40s
		GetSystemTime(&time);
		now = (double)time.wMinute * 60.0 + time.wSecond + (double)time.wMilliseconds / 1000.0;
		// Send directly velocity to motors (% of maximum voltage)
		if (WMOT > 0) {
			target = 100.0*sin((now - start) * 0.1);// play a sinus
			cout << "Velocity :" << target << "% (" << (now - start)  << "s)" << endl;
			pw[0].setvel(target);
			if (WMOT > 1)	pw[1].setvel(target);
		}

		/*
		// Position control with encoder feedback.
		pw[leftM].target=target1;	//set the target position
		pw[leftM].started_pos=true;	//start the PID
		if(WMOT>1)	pw[!leftM].target=target2;
		if(WMOT>1)	pw[!leftM].started_pos=true;*/

		/*
		// Current control with current feedback. !!Never tested!!
		pw[leftM].target=target2;
		pw[leftM].started_cur=true;
		if(WMOT>1)	pw[!leftM].target=target2;
		if(WMOT>1)	pw[!leftM].started_cur=true;*/

		/*
		// Run the stepper
		__int64 stepcommand = pw[0].rad2steps(target1);
		if (abs(stepcommand)>10000)	stepcommand = sgn(stepcommand) * 10000;
		pw[0].GoToStepper(0, stepcommand);*/

		//GetSystemTime(&time);
		//WORD now = ((time.wMinute * 60 + time.wSecond) * 1000) + time.wMilliseconds;
		csvfile << now-start << ";" << target << ";" << pw[0].curr_pos << "; " << pw[1].curr_pos << "; " << pw[0].vel_raw[9] << "; " << pw[1].vel_raw[9] << "; " << pw[0].vel_fil[9] << "; " << pw[1].vel_fil[9] << "; " << pw[0].curr_accel << "; " << pw[1].curr_accel << "; " << pw[0].cur_fil[9] << "; " << pw[1].cur_fil[9] << "; " << endl;

		Sleep(10);
	}

	/// CLOSING
	if (WMOT) {
		pw[0].closeMot();
		if (WMOT>1)	pw[1].closeMot();
	}

	if (REC)	csvfile.close();

	return 0;
}
