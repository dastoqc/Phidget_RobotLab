#include <iostream>
#include <fstream>
#include <time.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "PhidgetClass.h"

#include "stdafx.h"

using namespace std;

CPhidgetWrapper pw[2];

#define WMOT 2	//0,1,2

/**
* @function main
*/
int main(int argc, char** argv)
{
	double target1 = 0.0, target2 = 0.0;
	int leftM = 0;

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
	int i = 500;
	while (i<=1000) {

		// Send directly velocity to motors (% of maximum voltage)
		cout << "Velocity :" << (double)i / 10.0 << "%" << endl;
		pw[0].setvel((double)i/10.0);
		if (WMOT > 1)	pw[1].setvel(-(double)i/10.0);

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
		Sleep(100);
		i++;
	}
	Sleep(500);
	/// CLOSING
	if (WMOT) {
		pw[0].closeMot();
		if (WMOT>1)	pw[1].closeMot();
	}

	return 0;
}
