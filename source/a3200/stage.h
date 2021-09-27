// Aerotech A3200 stage class

#pragma once

#ifndef A3200STAGE
#define A3200STAGE

#include <iostream>
#include <fstream>
#include <sstream>
#include "A3200.h"

namespace stim {
	class A3200 {
	private:
		A3200Handle handle;	// handle variable for A3200 functions
		float x_speed;	// x-drive translation speed
		float y_speed;	// y-drive translation speed
		float z_speed;	// z-drive translation speed

	public:
		A3200();	// constructor
		~A3200();	// destructor

		int connect();		// initialize and connect to stage
		void disconnect();	// disconnect to stage
		void perror();		// print out stage error messages
		void set_speed(AXISINDEX idx, float value);		// set axis translation velcities
		int home(AXISMASK midx);	// home process
		int moveto(DOUBLE *origin);	// translate to position in the xy-plane
		int moveto(AXISMASK midx, DOUBLE position);		// translate an axis to position
		int moveby(AXISINDEX idx, DOUBLE distance);		// translate an axis by distance
		int read_position(int idx, DOUBLE &position);	// read current position along one axis
	};
}

#endif