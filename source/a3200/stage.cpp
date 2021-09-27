#include "stage.h"

namespace stim {
	A3200::A3200() {	// default constructor -- set axis translation velocities
		handle = NULL;
		x_speed = 3.0f; y_speed = 1.5f; z_speed = 0.01f;	// Jack's magic numbers
	}

	A3200::~A3200() {
		handle = NULL;
	}

	int A3200::connect() {
		std::cout << "connecting to A3200..." << std::endl;
		if (!A3200Connect(&handle)) { perror(); return 1; }	// connect to A3200 stage and enable a handle
		else std::cout << "A3200 connected..." << std::endl;

		if (!A3200MotionEnable(handle, TASKID_01, AXISMASK_00)) { perror(); return 1; }	// enable x-axis
		if (!A3200MotionEnable(handle, TASKID_01, AXISMASK_01)) { perror(); return 1; }	// enable y-axis
		if (!A3200MotionEnable(handle, TASKID_01, AXISMASK_02)) { perror(); return 1; }	// enable z-axis

		//if (!A3200Reset(handle)) { perror(); return 1; }	// may reset controller

		return 0;
	}

	void A3200::disconnect() {
		if (NULL != handle) {
			if (!A3200MotionDisable(handle, TASKID_01, AXISMASK_00)) { perror(); }	// disable x-axis
			if (!A3200MotionDisable(handle, TASKID_01, AXISMASK_01)) { perror(); }	// disable y-axis
			if (!A3200MotionDisable(handle, TASKID_01, AXISMASK_02)) { perror(); }	// disable z-axis
			if (!A3200Disconnect(handle)) { perror(); }
		}
	}

	void A3200::perror() {
		CHAR data[1024];
		A3200GetLastErrorString(data, 1024);	// read last error string for print
		std::cout << data << std::endl;
		std::string filename = "error.txt";		// output last error string to text
		std::ofstream file;
		file.open(filename);
		file << "stage error: " << data;
		file.close();
	}

	void A3200::set_speed(AXISINDEX idx, float value) {	// set axis translation velocity in mm/s
		if (idx == AXISINDEX_00)
			x_speed = value;
		else if (idx == AXISINDEX_01)
			y_speed = value;
		else if (idx == AXISINDEX_02)
			z_speed = value;
	}

	int A3200::home(AXISMASK midx) {
		if (!A3200MotionSetupAbsolute(handle, TASKID_01)) { perror(); return 1; }	// switch to ABSOLUTE mode
		if (!A3200MotionHome(handle, TASKID_01, midx)) { perror(); return 1; }
		if (!A3200MotionWaitForMotionDone(handle, midx, WAITOPTION_MoveDone, -1, NULL)) { perror(); return 1; }	// wait until motion done
		//Sleep(5000);		// pause the system during translation

		return 0;
	}

	int A3200::moveto(DOUBLE *origin) {	// overload function MOVETO: translate stage to a preset origin along the xy-plane
		if (!A3200MotionSetupAbsolute(handle, TASKID_01)) { perror(); return 1; }	// switch to ABSOLUTE mode
		float speed = std::fminf(x_speed, y_speed);
		if (!A3200MotionLinearVelocity(handle, TASKID_01, (AXISMASK)(AXISMASK_00 | AXISMASK_01), origin, (DOUBLE)speed)) { perror(); return 1; }	// translate to origin
		if (!A3200MotionWaitForMotionDone(handle, (AXISMASK)(AXISMASK_00 | AXISMASK_01), WAITOPTION_MoveDone, -1, NULL)) { perror(); return 1; }	// wait until motion done
		//Sleep(1500);		// pause the system during translation

		return 0;
	}

	int A3200::moveto(AXISMASK midx, DOUBLE position) {	// overload function MOVETO: translate stage to a preset position along one axis
		if (!A3200MotionSetupAbsolute(handle, TASKID_01)) { perror(); return 1; }	// switch to ABSOLUTE mode
		float speed;
		if (midx == AXISMASK_00)
			speed = x_speed;
		else if (midx == AXISMASK_01)
			speed = y_speed;
		else if (midx == AXISMASK_02)
			speed = z_speed;
		if (!A3200MotionLinearVelocity(handle, TASKID_01, midx, &position, (DOUBLE)speed)) { perror(); return 1; }	// translate to position
		if (!A3200MotionWaitForMotionDone(handle, midx, WAITOPTION_MoveDone, -1, NULL)) { perror(); return 1; }		// wait until motion done
		//Sleep(1500);		// pause the system during translation

		return 0;
	}

	int A3200::moveby(AXISINDEX idx, DOUBLE distance) {
		if (!A3200MotionSetupIncremental(handle, TASKID_01)) { perror(); return 1; }// switch to INCREMENTAL mode
		float speed;
		AXISMASK midx;
		if (idx == AXISINDEX_00) {
			speed = x_speed;
			midx = AXISMASK_00;
		}
		else if (idx == AXISINDEX_01) {
			speed = y_speed;
			midx = AXISMASK_01;
		}
		else {
			speed = z_speed;
			midx = AXISMASK_02;
		}

		if (!A3200MotionMoveInc(handle, TASKID_01, idx, distance, (DOUBLE)speed)) { perror(); return 1; }		// translate along x-axis by xssize, speed default to 1
		if (!A3200MotionWaitForMotionDone(handle, midx, WAITOPTION_MoveDone, -1, NULL)) { perror(); return 1; }	// wait until motion done
		//Sleep(1500);		// pause the system during translation

		return 0;
	}

	int A3200::read_position(int idx, DOUBLE &position) {
		if (!A3200StatusGetItem(handle, idx, STATUSITEM_PositionFeedback, 0, &position)) { perror(); return 1; }

		return 0;
	}
}