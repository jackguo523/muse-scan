// Large-scale MUSE scanning
// Author: Jiaming Guo, STIM lab, University of Houston, 02/21/2020
// please use it together with Image_Stitching.exe


// LIBRARY INCLUDE
// STL include
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include "windows.h"

// STIM include
#include <stim/parser/arguments.h>
#include <stim/ui/progressbar.h>

// 3rd Party include
#include "tsi/tl_camera_sdk.h"
#include "tsi/tl_camera_sdk_load.h"
#include "tsi/tl_color_enum.h"
#include "tsi/tl_color_demosaic_load.h"
#include "tsi/tl_color_processing_load.h"
#include "tsi/tl_mono_to_color_processing_load.h"
#include "tsi/tl_mono_to_color_processing.h"

// Project include
#include "tsi/thorcam.h"
#include "a3200/stage.h"
#include "metric/fmeasure.h"
#include "timer.h"


// GLOBAL VARIABLES
stim::arglist args;		// user arguments
std::string user = "";	// user name

float bx0 = 0.0f; float by0 = 0.0f;					// scan origin coordinates (top-left) in mm, often set to (0,0)
float bsx = 1.0f; float bsy = 1.0f;					// scan bounding box area in mm
float totalx = 0.0f; float totaly = 0.0f;			// actual total scan area in mm
float psize = 0.0f;									// lateral pixel size or sampling rate in um/pixel
float FX = 0.0f; float FY = 0.0f;					// field of view in mm
int xstep = 0; int ystep = 0;						// lateral scan count
float xssize = 0.0f; float yssize = 0.0f;			// lateral step size in mm
float overlap = 0.0f;								// overlap rate in %
int cam_expo = 0; int cam_gain = 0; int cam_bl = 0;	// camera settings
std::string output_dir = "";						// output directory
std::string format;									// output format
bool thread = false;								// flag indicates cpu multi-threading
bool demosaic = false;								// flag indicates raw image demosaicking
bool compression = false;							// flag indicates bit depth compression
int totalI = 0;										// total numbers of frames to collect
int countI = 0;										// index of current frame
int mode = 1;										// autofocus mode, default to quick scan
int fmm = 1;										// focus measure metric, default to glsd
unsigned int p = 0;									// scan progress in %
int prange = 0; int nrange = 0;						// positive and negative ranges in um, assumed to be equal
int zssize = 0;										// z-drive step size in um
int pcount = 0; int ncount = 0;						// z-drive positive and negative current count
int psum = 0; int nsum = 0;							// z-drive positive and negative total count
float previous_fm = 0.0f;							// last focus measure value
float current_fm = 0.0f;							// current focus measure value
bool reach_limit = false;							// flag indicates out-of-range in autofocus
int move_count = 0;									// current step count along one arm in autofocus, reset for another arm
DOUBLE default_position = 0.0;						// default z-drive position (start z-position)
DOUBLE current_position = 0.0;						// current z-drive position
DOUBLE optimal_position = 0.0;						// optimal z-drive position (end z-position)
DOUBLE inter_position = 0.0;						// internal z-drive default position between tiles
std::vector<DOUBLE> op;								// optimal z-drive positions

std::chrono::seconds itime;							// acquisition time

// GLOBAL FUNCTIONS
// Print help
void advertise() {
	std::cout << std::endl << std::endl;
	std::cout << " ==========================================================" << std::endl;
	std::cout << "|                 Large-scale MUSE Scanning                |" << std::endl;
	std::cout << "|                  Developer: Jiaming Guo		   |" << std::endl;		// can not believe this happen :<
	std::cout << " ==========================================================" << std::endl << std::endl;
	std::cout << "example command: muse-scan --user Jack --box 0 0 10 10 --res 0.325 --overlap 5 --cam 100 0 0 --format tif --dir test --thread --compression --mode 1 --fmeasure 1 --zrange 100 --zssize 1" << std::endl << std::endl << std::endl;
	// current patch uses snake scan (right & down)
	// --------------------->----------------------
	//                                            |
	// ---------------------<----------------------
	// |
	// --------------------->----------------------
	// x-incremental in odd rows and x-decremental in even rows, y-decremental in all columns
	std::cout << args.str();	// print all arguments
}
// prograssbar update
void pupdate(int &cur, int tot) {
	p = (unsigned int)((cur * 100) / tot);
	rtsProgressBar(p);
}
// collect a frame
void collect(stim::thorcam cam, int &c, std::string suffix = "") {
	cam.fire();			// collect a frame from buffer
	c++;				// frame count increment
	cam.save(c, suffix);// save current frame to disk
}
// read input arguments
void read_args() {
	// read user name as record
	user = args["user"].as_string();
	// read scan bounding box information (top-left points, bounding box size), default in mm for any A3200 translation
	bx0 = (float)args["box"].as_float(0); by0 = (float)args["box"].as_float(1); bsx = (float)args["box"].as_float(2); bsy = (float)args["box"].as_float(3);
	// read image lateral pixel size, default in um/pixel and compute total field of view in mm
	psize = (float)args["res"].as_float();
	FX = (float)(psize * width / 1000.f); FY = (float)(psize * height / 1000.f);
	// read overlap rate and convert to decimals
	overlap = (float)(args["overlap"].as_float() / 100.0f);
	// compute the lateral step size based on equation $$$ceil[(tissue size - field of view)/(field of view * (1 - overlap))]$$$ and compute the total number of images and actual scan box
	xstep = (int)ceil((bsx - FX) / (FX * (1.0f - overlap))); ystep = (int)ceil((bsy - FY) / (FY * (1.0f - overlap)));
	if (xstep < 0) xstep = 0; if (ystep < 0) ystep = 0;	 // account for the single field of view case
	totalI = (xstep + 1) * (ystep + 1);	// update the total number of images
	totalx = (float)xstep * FX * (1.0f - overlap) + FX; totaly = (float)ystep * FY * (1.0f - overlap) + FY;	// update the actual scan box
	xssize = FX * (1.0f - overlap); yssize = FY * (1.0f - overlap);	// compute the lateral step size in mm as the stage receives mm information
	// read all camera firing related information
	cam_expo = args["cam"].as_int(0); cam_gain = args["cam"].as_int(1); cam_bl = args["cam"].as_int(2);
	format = args["format"].as_string();
	output_dir = args["dir"].as_string();
	thread = args["thread"].is_set();
	demosaic = args["demosaic"].is_set();
	compression = args["compression"].is_set();
	// read autofocus mode and focus measure metric
	mode = args["mode"].as_int(); 
	if (mode <= 0 || mode > 3) {
		std::cout << "please specify autofocus mode as integer in range [1,3]" << std::endl;
		std::exit(1);
	}
	fmm = args["fmeasure"].as_int();
	if (fmm <= 0 || fmm > 4) {
		std::cout << "please specify focus measure metric as integer in range [1,4]" << std::endl;
		std::exit(1);
	}
	// read z-drive related parameters for autofocus
	int range = args["zrange"].as_int();
	prange = range; nrange = range;	// assume the same positive and negative ranges
	zssize = args["zssize"].as_int();
	psum = prange / zssize; nsum = nrange / zssize;	// compute total z-drive counts
}
// perform autofocus based on global focus measure along one arm (-1/+1 z-drive), default to negative arm
int zfocus(stim::thorcam &cam, stim::A3200 &a3200, int direction = -1) {
	pcount = 0; ncount = 0;	// reset positive and negative current count for one arm
	
	do {
		if (a3200.read_position(2, current_position)) return 1;	// read current z-drive position
		optimal_position = current_position;	// set optimal position to initial position, !this line can be deleted!

		cam.fire();					// collect a frame from buffer
		previous_fm = current_fm;	// update previous focus measure to current value
		if (cam.d_compression)		// update current focus measure
			current_fm = fm::eval_fm<unsigned char>(cam.output_buffer_24, width, height, (fm::fmetric)fmm);	// 24bit
		else
			current_fm = fm::eval_fm<unsigned short>(cam.output_buffer, width, height, (fm::fmetric)fmm);	// 48bit

		if (a3200.moveby(AXISINDEX_02, (DOUBLE)(direction * zssize / 1000.0))) return 1;	// translate z-drive up or down
		pcount++; ncount++; move_count++;	// step count increment

		if (direction == 1) {	// check for boundary case in positive arm
			if (pcount == psum) {
				if (current_fm >= previous_fm) {
					reach_limit = true;
					std::cout << "positive limit reached, please specify larger autofocus range" << std::endl;
				}
				break;	// break if positive limit reached
			}
		}
		else {	// check for boundary case in negative arm
			if (ncount == nsum) {
				if (current_fm >= previous_fm) {
					reach_limit = true;
					std::cout << "negative limit reached, please specify larger autofocus range" << std::endl;
				}
				break;	// break if negative limit reached
			}
		}
		if (current_fm >= previous_fm)
			optimal_position = current_position;	// update optimal position to current value to generate height map
	} while (current_fm >= previous_fm);

	return 0;
}
// perform global autofocus scan
int autofocus(stim::thorcam &cam, stim::A3200 &a3200) {
	move_count = 0;	// reset autofocus translation count for good-roughness scan
	previous_fm = 0.0f;	current_fm = 0.0f;	// reset history focus measure values

	if (zfocus(cam, a3200, -1)) return 1;	// scan first along negative arm to avoid potential collision
	if (move_count < 3) {	// two possible cases: (1) optimal position exists in positive arm or (2) default position is optimal
		a3200.moveto(AXISMASK_02, (DOUBLE)inter_position);			// reset to internal default position for second arm
		previous_fm = 0.0f;	current_fm = 0.0f;	// reset focus measure values
		if (zfocus(cam, a3200, 1)) return 1;// scan then along positive arm
	}

	op.push_back(optimal_position);	// push back optimal position to list
	if (a3200.moveto(AXISMASK_02, (DOUBLE)optimal_position)) return 1;	// set to optimal position
	collect(cam, countI);	// collect a frame
	pupdate(countI, totalI);// update progress bar

	return 0;
}
// perform z-traverse for fusion
int ztraverse(stim::thorcam &cam, stim::A3200 &a3200, int row, int col) {
	if (a3200.moveto(AXISMASK_02, (DOUBLE)default_position)) return 1;		// reset to default position
	collect(cam, countI);	// collect ground truth
	if (a3200.moveby(AXISINDEX_02, (DOUBLE)(-nrange / 1000.0))) return 1;	// set to minimum position to start z-drive streaming
	
	int ssum = psum + nsum + 1;	// compute streaming total count
	//  || +2 ||
	//  || +1 ||
	//  || 0  || --> origin
	//  || -1 ||
	//  || -2 ||
	int scount = 0;				// reset streaming current count
	std::stringstream ssuffix;	// create streaming suffix
	ssuffix << "/FOV(" << row << "," << col << ")";
	// (0,0) (0,1) (0,2)
	// (1,0) (1,1) (1,2)
	// (2,0) (2,1) (2,2)
	
	for (int d = 0; d < ssum; d++) {
		collect(cam, scount, ssuffix.str());	// collect a frame and then translate z-drive produces the exactly numbers of frames requested
		if (a3200.moveby(AXISINDEX_02, (DOUBLE)(zssize / 1000.0))) return 1;	// translate z-drive up or down
	}

	return 0;
}
// Large-scale muse scan
int scan(stim::thorcam &cam, stim::A3200 &a3200, int mode = 1) {
	DOUBLE origin[2] = { (DOUBLE)bx0, (DOUBLE)by0 };	// retrieve lateral origin coordinates

	// note that this origin is often manually set to (0, 0) in the system by resetting the stage before acquisition
	if (a3200.moveto(origin)) return 1;		// home to the lateral origin
	if (a3200.read_position(2, default_position)) return 1;		// read current z-drive position, should be 0.0mm after reset

	pupdate(countI, totalI);	// update progress
	int i = 0;	// x-drive count
	int j = 0;	// y-drive count

	for (; j < ystep + 1; j++) {	// outer loop for y-drive
		if (a3200.read_position(2, inter_position)) return 1;		// read current z-drive position for autofocus for each tile
		i = 0;	// reset x-drive count

		if (mode == 1) {	// for quick scan
			collect(cam, countI);	// collect a frame
			pupdate(countI, totalI);// update progress bar
		}
		else if (mode == 2) {	// for good-roughness scan
			autofocus(cam, a3200);
		}
		else if (mode == 3) {	// for comprehensive scan
			if (ztraverse(cam, a3200, j, i)) return 1;
			pupdate(countI, totalI);// update progress bar
		}

		int xfactor = (j % 2) == 0 ? 1 : -1;	// x-drive translation coefficient, (row j) even means right (positive) while odd means left (negative) in snake scan

		for (i = 1; i < xstep + 1; i++) {
			if (a3200.read_position(2, inter_position)) return 1;		// read current z-drive position as default, should be 0.0mm
			if (a3200.moveby(AXISINDEX_00, (DOUBLE)xfactor * xssize)) return 1;
			//if (a3200.read_position(2, current_position)) return 1;		// read current z-drive position
			//std::cout << "move along x-direction by " << xfactor * xssize << std::endl;

			if (mode == 1) {
				collect(cam, countI);	// collect a frame
				pupdate(countI, totalI);// update progress bar
			}
			else if (mode == 2) {
				if (autofocus(cam, a3200)) return 1;
			}
			else if (mode == 3) {
				if (ztraverse(cam, a3200, j, i)) return 1;
				pupdate(countI, totalI);// update progress bar
			}
		}

		if (j != ystep)		// no row translation for final row
			if (a3200.moveby(AXISINDEX_01, (DOUBLE)-yssize)) return 1;																													
	}

	if (a3200.moveto(origin)) return 1;		// reset to the origin
	if (a3200.moveto(AXISMASK_02, (DOUBLE)default_position)) return 1;		// reset to default z-drive position

	return 0;
}
// saving process logs in disk
void log(int mode = 1) {
	std::string filename = "log.txt";
	std::stringstream ss;
	ss << output_dir << "/" << filename;
	std::ofstream file;
	file.open(ss.str());

	// read current time
	time_t rawtime;
	struct tm *timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	file << "process user: " << user << std::endl;
	file << "process date: " << asctime(timeinfo) << std::endl;
	file << "demand scan area: " << bsx << "x" << bsy << "mm" << std::endl;
	file << "actual scan area: " << totalx << "x" << totaly << "mm" << std::endl;
	file << "mosaic grid: " << (ystep + 1) << "x" << (xstep + 1) << std::endl;
	file << "tile overlap: " << overlap * 100 << "%" << std::endl;
	file << "exposure time: " << cam_expo << "ms, gain: " << cam_gain << ", black level: " << cam_bl << std::endl;

	file << "frame:" << width << "x" << height << std::endl;
	file << "pixel size: " << psize << "um/pixel" << std::endl;
	file << "acquisition time: " << itime.count() << "s" << std::endl;

	if (mode == 1) {			// for quick scan
		file << std::endl;
		file << "QUICK SCAN" << std::endl;
	}
	else if (mode == 2) {		// for good-roughness scan
		file << std::endl;
		file << "ROOD-ROUGHNESS SCAN" << std::endl;
		file << "auto focus z-drive range: [" << -nrange << ", " << prange << "]um, z-drive stepsize: " << zssize << "um" << std::endl;
		file << "default z-drive position: " << std::fixed << std::setprecision(5) << (float)default_position << "mm" << std::endl;
		file << "auto focus z-map: " << std::endl;
		int subx = xstep + 1; int suby = ystep + 1;
		for (int j = 0; j < suby; j++) {
			for (int i = 0; i < subx; i++) {
				int id = j * subx;		// convert from snake coordinate to normal coordinate
				if (j % 2 == 0)
					id += i;
				else
					id += subx - 1 - i;
				file << std::fixed << std::setprecision(5) << (float)op[id];
				if (i != subx + 1)
					file << "   ";
			}
			if (j != suby + 1)
				file << std::endl;
		}
	}
	else if (mode == 3) {
		file << std::endl;
		file << "COMPREHENSIVE SCAN" << std::endl;
		file << "auto focus z-drive range: [" << -nrange << ", " << prange << "]um, z-drive stepsize: " << zssize << "um" << std::endl;
		file << "total z-stream count: " << psum + nsum + 1 << std::endl;
	}

	file.close();
}


int main(int argc, char* argv[]) {

	//std::string file_mask = "*.tif";
	//stim::filename file_path(file_mask);							//get the path for the images
	//std::vector<stim::filename> file_list = file_path.get_list();	//get the list of files
	//std::vector<std::string> string_list(file_list.size());
	//std::vector<float> FM; float tmp = 0.0f;
	//for (int i = 0; i < file_list.size(); i++) {
	//	string_list[i] = file_list[i].str();
	//	stim::image<unsigned char> I(string_list[i]);
	//	tmp = fm::eval_fm<unsigned char>(I.data(), width, height, (fm::fmetric)1);
	//	FM.push_back(tmp);
	//}

	// add user-defined arguments
	args.add("help", "print the usage help");
	args.add("user", "user name as record", "Jack", "any valid user from STIM");							// specify the operation dude, default to Jack
	args.add("box", "define the scan box (x0, y0, sx, sy) in mm", "0 0 1 1", "four real values > 0");		// specify the scan box in a rect format (top-left-x, top-left-y, bb-x, bb-y)
	args.add("res", "resolution or pixel size in um/pixel", "0.392", "real value > 0");						// specify the effective lateral sampling rate, default to 0.392um/pixel for the Nikon 10X objective
	args.add("overlap", "mosaic overlap rate in %", "5", "real value > 0");									// specify the overlap rate in percentage, recommend to [5~15]
	args.add("cam", "camera parameters (exposure, gain, black level)", "100 5 0", "three real values > 0");	// specify the camera parameters including exposure time in ms, gain in dB, and black level no unit
	args.add("format", "output image format", "tif", "any valid image format, ex. tif, png, bmp");			// specify the output image format, ex. bmp, png, tif, pgm, ppm, jpg
	args.add("dir", "output directory", "result", "any valid directory, ex. stim/desktop/result");			// specify the output directory, if not exist, create one
	args.add("thread", "activate cpu multi-threading for frame buffering");									// specify the flag to use either callback function on a worker thread or polling function on the main thread
	args.add("demosaic", "demosaic the raw image based on the Bayer pattern");								// specify to use either advanced or basic demosaicking to generate rgb images from raw images
	// the Bayer pattern
	// ---------------
	// |      |      |
	// |  R   |  GR  |
	// |      |      |
	// ---------------
	// |      |      |
	// |  GB  |  B   |
	// |      |      |
	// ---------------     R = a red pixel, GR = a green pixel next to a red pixel, B = a blue pixel, GB = a green pixel next to a blue pixel.
	args.add("compression", "image compression: 8bit, 16bit");												// specify to apply either 8bit or 16bit bit depth
	args.add("mode", "autofocus mode: quick, good-roughness, comprehensive", "1", "any integer in [1,3]");	// specify the autofocus mode: 1->quick scan, 2->good-roughness scan, 3->comprehensive scan, default to 1->quick scan
	// quick scan: scan without autofocus -- good for mounted tissue sections
	// good-roughness scan: scan with frame-level autofocus -- good for embedded tissue blocks
	// comprehensive scan: scan with a stack of height map for each frame -- good for fresh tissue or biopsy
	args.add("fmeasure", "focus measure metric: GLSD, SPFQ, BREN, HISE", "1", "any interger in [1,4]");		// specify the focus measure metric: 1->GLSD, 2->SPFQ, 3->BREN, 4->HISE, default to GLSD
	// GLSD->grayscale standard deviation, SPFQ->spatial frequency, BREN->Brenner's first differentiation, HISE->histogram entropy
	args.add("zrange", "define the z-drive travel distance along one arm in um", "50", "real value > 0");	// specify the z-drive travel distance along one direction, default to 50um for the Nikon 10X objective (in total 50um considering positive and negative parts)
	args.add("zssize", "define the z-step size in um", "1", "real value > 0");								// specify the z-drive step size, default to 1um for the Nikon 10X objective
	// The lateral sampling rate is determined by the microscope while the axial sampling rate is simply determined by the z-drive step size
	

	args.parse(argc, argv);	// parse the command line
	if (args["help"].is_set()) {
		advertise();	// print usage help and all arguments
		std::exit(1);
	}
	read_args();		// read all user input parameters

	stim::thorcam cam(thread, demosaic, compression);	// create a thorlabs camera object
	if (cam.connect(cam_expo, cam_gain, cam_bl, output_dir, format)) { cam.disconnect(); std::exit(1); }	// connect to camera via the created camera object
	if (cam.configure()) { cam.disconnect(); std::exit(1); }	// configure camera
	stim::A3200 a3200;									// create a A3200 stage object
	if (a3200.connect()) { a3200.disconnect(); std::exit(1); }	// connect to stage via the created stage object

	// hmm.....
	system("CLS");	// print start point
	float totalMem = (xstep + 1) * (ystep + 1) * (psum + nsum + 1 + 1) * 25.3 / 1024.0f;	// each image requires 25.3MB memory
	std::cout << totalMem << "GB memory is required" << std::endl << std::endl;	// pop ups a warning sign to remind users to check for memory
	std::cout << "START ACQUISITION";
	Sleep(500); std::cout << "."; Sleep(500); std::cout << "."; Sleep(500); std::cout << "."; Sleep(500); std::cout << "."; Sleep(500); std::cout << ".";	// animation
	std::cout << std::endl; Sleep(500);

	timer_start();		// timer starts
	if (scan(cam, a3200, mode)) { cam.disconnect(); a3200.disconnect(); std::exit(1); }	// perform large-scale scan
	std::cout << std::endl << "END ACQUISITION....." << std::endl;
	itime = timer_stop<std::chrono::seconds>();	// timer stops, in seconds
	std::cout << "it takes " << itime.count() << "s to process" << std::endl;
	
	log(mode);			// output logs

	cam.disconnect();	// disconnect to camera
	a3200.disconnect();	// disconnect to stage

	std::cout << "press any key to exit" << std::endl;
	std::cin.get();

	return 0;
}
