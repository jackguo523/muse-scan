// Thorlabs CMOS-series camera class

#pragma once

#ifndef THORCAM_H
#define THORCAM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "windows.h"
#include <stim/image/image.h>
#include "tl_camera_sdk.h"
#include "tl_camera_sdk_load.h"
#include "tl_color_enum.h"
#include "tl_color_demosaic_load.h"
#include "tl_color_processing_load.h"
#include "tl_mono_to_color_processing_load.h"
#include "tl_mono_to_color_processing.h"

extern int width; extern int height;	// camera frame width and height
void frame_available_callback(void* sender, unsigned short* image_buffer, int frame_count, unsigned char* metadata, int metadata_size_in_bytes, void* context);

namespace stim {
	class thorcam {
	private:
		int is_camera_dll_open;					// camera dll flag
		int is_camera_sdk_open;					// camera sdk flag
		void *camera_handle;					// camera handle
		int is_color_processing_sdk_open;		// color processing sdk flag
		int is_demosaic_sdk_open;				// demosaic sdk flag
		int is_mono_to_color_sdk_open;			// mono to color processing sdk flag
		void *color_processor_handle;			// color processor handle
		void *mono_to_color_processor_handle;	// mono to color processor handle

		enum TL_CAMERA_SENSOR_TYPE camera_sensor_type;				// camera sensor type
		enum TL_COLOR_FILTER_ARRAY_PHASE color_filter_array_phase;	// color filter array phase -- color pattern
		float color_correction_matrix[9];							// color correction matrix append
		float default_white_balance_matrix[9];						// default while balance matrix append
		int bit_depth;												// image bit size

	protected:
		int exposure;			// camera exposure time in ms
		int gain;				// camera digital gain in dB
		int black_level;		// camera black level no unit
		std::string output_dir;	// camera output directory
		std::string format;		// camera output format

	public:
		bool d_thread;		// device multi-threading flag
		bool d_demosaic;	// device demosaicking flag
		bool d_compression;	// device compression (8/16) flag

		unsigned short *output_buffer;		// output frame buffer in 48bit
		unsigned char *output_buffer_24;	// output frame buffer in 24bit
		unsigned short *demosaic_buffer;	// demosaic frame buffer

		thorcam();		// default constructor
		thorcam(bool thread, bool demosaic, bool compression);		// copy constructor
		~thorcam();		// destructor

		int connect(int expo, int gn, int bl, std::string odir, std::string fmt);	// initialize and connect to camera
		int configure();	// configure camera
		void disconnect();	// disconnect to camera
		void fire();		// collect a frame
		void save(int count, std::string suffix = "");	// save current frame
	};
}

#endif