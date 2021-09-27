#include "thorcam.h"

static float home_color_correction[9] = { 1.25477f , -0.15359f , -0.10118f , -0.07011f , 1.13723f , -0.06713f , 0.0f , -0.26641f , 1.26641f };
static float default_color_correction[9] = { 1.3f , -0.3f , 0.0f , -0.45f , 1.55f , -0.1f , 0.0f , -0.2f , 1.2f };
static float home_white_balance[9] = { 1.0f , 0.0f , 0.0f , 0.0f , 1.37f, 0.0f , 0.0f , 0.0f , 2.79f };
static float default_white_balance[9] = { 2.63f , 0.0f , 0.0f , 0.0f , 1.0f, 0.0f , 0.0f , 0.0f , 3.41f };

HANDLE frame_acquired_event = 0;					// multi thread handle
volatile int is_first_frame_finished = 1;			// first frame flag, very sensitive to threading
unsigned short *callback_image_buffer_copy = 0;		// callback function frame buffer
unsigned short *poll_image_buffer_copy = 0;			// poll function frame buffer
int width = 4096; int height = 2160;				// frame width and height

void frame_available_callback(void* sender, unsigned short* image_buffer, int frame_count, unsigned char* metadata, int metadata_size_in_bytes, void* context) {
	if (is_first_frame_finished)
		return;

	is_first_frame_finished = 1;

	if (!frame_acquired_event)
		SetEvent(frame_acquired_event);

	if (callback_image_buffer_copy)
		memcpy(callback_image_buffer_copy, image_buffer, (sizeof(unsigned short) * width * height));
}

namespace stim {
	thorcam::thorcam() {
		is_camera_dll_open = 0;
		is_camera_sdk_open = 0;
		camera_handle = 0;
		is_color_processing_sdk_open = 0;
		is_demosaic_sdk_open = 0;
		is_mono_to_color_sdk_open = 0;
		color_processor_handle = 0;
		mono_to_color_processor_handle = 0;

		output_buffer = 0;
		output_buffer_24 = 0;
		demosaic_buffer = 0;

		exposure = 20;
		gain = 0;
		black_level = 0;

		d_thread = true;		// default to cpu multi-threading
		d_demosaic = false;		// default basic demosaic
		d_compression = true;	// default to compression 8bit
	}

	thorcam::thorcam(bool thread, bool demosaic, bool compression) {
		is_camera_dll_open = 0;
		is_camera_sdk_open = 0;
		camera_handle = 0;
		is_color_processing_sdk_open = 0;
		is_demosaic_sdk_open = 0;
		is_mono_to_color_sdk_open = 0;
		color_processor_handle = 0;
		mono_to_color_processor_handle = 0;

		output_buffer = 0;
		output_buffer_24 = 0;
		demosaic_buffer = 0;

		exposure = 20;
		gain = 0;
		black_level = 0;

		d_thread = thread;
		d_demosaic = demosaic;
		d_compression = compression;
	}

	thorcam::~thorcam() {
	}

	int thorcam::connect(int expo, int gn, int bl, std::string odir, std::string fmt) {
		std::cout << "loading camera dll..." << std::endl;
		if (tl_camera_sdk_dll_initialize()) { std::cout << "failed to initialize dll" << std::endl; return 1; }	// dynamically load sdk and dll and get handles to its exported functions
		std::cout << "camera dll loaded..." << std::endl;
		is_camera_dll_open = 1;

		std::cout << "opening camera sdk..." << std::endl;
		if (tl_camera_open_sdk()) { std::cout << "failed to open sdk" << std::endl; return 1; }	// initialize sdk packs
		std::cout << "camera sdk opened..." << std::endl;
		is_camera_sdk_open = 1;

		char cam_id[1024];
		cam_id[0] = 0;	// only use the first discovered camera for imaging
		std::cout << "discovering available camera..." << std::endl;
		if (tl_camera_discover_available_cameras(cam_id, 1024)) { std::cout << "failed to discover available camera" << std::endl; return 1; }	// discover the serial numbers of all connected cameras
		std::cout << "camera discovered..." << std::endl;

		std::cout << "connecting to camera..." << std::endl;
		if (!strlen(cam_id)) { std::cout << "no available camera" << std::endl; return 1; }
		char *p_space = strchr(cam_id, ' ');
		if (p_space) *p_space = '\0';		// isolate the first detected camera
		char first_cam[256];
		strcpy_s(first_cam, 256, cam_id);	// get the camera name
		if (tl_camera_open_camera(first_cam, &camera_handle)) { std::cout << "failed to connect to camera #" << first_cam << std::endl; return 1; }	// open a connection to a camera of choice and get a handle of it
		std::cout << "camera #" << first_cam << " connected..." << std::endl;

		if (tl_camera_set_exposure_time(camera_handle, (long long)(expo * 1000))) { std::cout << "failed to set exposure time" << std::endl; return 1; }	// convert from mm to um, 1000 -> 1ms
		if (tl_camera_set_gain(camera_handle, gn)) { std::cout << "failed to set digital gain" << std::endl; return 1; }
		if (tl_camera_set_black_level(camera_handle, bl)) { std::cout << "failed to set black level" << std::endl; return 1; }
		output_dir = odir;
		format = fmt;

		if (tl_camera_get_camera_sensor_type(camera_handle, &camera_sensor_type)) { std::cout << "failed to get camera sensor type" << std::endl; return 1; };							// find camera sensor type
		if (tl_camera_get_color_filter_array_phase(camera_handle, &color_filter_array_phase)) { std::cout << "failed to get camera color filter array phase" << std::endl; return 1; }	// find the reference color
		if (tl_camera_get_color_correction_matrix(camera_handle, color_correction_matrix)) { std::cout << "failed to get color correction matrix" << std::endl; return 1; }				// find color correction matrix
		if (tl_camera_get_default_white_balance_matrix(camera_handle, default_white_balance_matrix)) { std::cout << "failed to get while balance matrix" << std::endl; return 1; }		// find default white balance matrix
		if (tl_camera_get_bit_depth(camera_handle, &bit_depth)) { std::cout << "failed to get bit depth" << std::endl; return 1; }														// find image bit size
		if (camera_sensor_type != TL_CAMERA_SENSOR_TYPE_BAYER) { std::cout << "camera is not a color camera" << std::endl; return 1; }	// check viability for color imaging

		if (tl_camera_get_image_width(camera_handle, &width)) { std::cout << "failed to get image width" << std::endl; return 1; }		// get the camera sensor block width
		if (tl_camera_get_image_height(camera_handle, &height)) { std::cout << "failed to get image height" << std::endl; return 1; }	// get the camera sensor block height

		callback_image_buffer_copy = new unsigned short[width * height];// allocate memory for callback image buffer copy
		poll_image_buffer_copy = new unsigned short[width * height];	// allocate memory for poll image buffer copy
		output_buffer = new unsigned short[width * height * 3];			// allocate memory for output color image
		demosaic_buffer = new unsigned short[width * height * 3];		// allocate memory for temporary buffer to store demosaic result
		output_buffer_24 = new unsigned char[width * height * 3];		// allocate memory for output_24 color image

		return 0;
	}

	int thorcam::configure() {

		if (d_demosaic) {	// using bayer filter pattern to demosaic a raw monochrome image
			if (tl_color_processing_initialize()) { std::cout << "failed to initialize color processing dll and sdk" << std::endl; return 1; }	// initialize color processing dll and sdk
			is_color_processing_sdk_open = 1;
			if (tl_demosaic_initialize()) { std::cout << "failed to initialize demosaic dll and sdk" << std::endl; return 1; }					// initialize demosaic dll and sdk
			is_demosaic_sdk_open = 1;

			color_processor_handle = tl_color_create_color_processor(bit_depth, bit_depth);	// construct a color processor
			if (!color_processor_handle) { std::cout << "failed to construct a color processor" << std::endl; return 1; }
			// append 3x3 matrix for correction multiplication, the order matters
			tl_color_append_matrix(color_processor_handle, default_white_balance);
			tl_color_append_matrix(color_processor_handle, default_color_correction);
			// use RGB LUTs to configure an sRGB nonlinear function -- need to create own LUT
			sRGB_companding_LUT(bit_depth, tl_color_get_red_output_LUT(color_processor_handle));	// follow RGB order as previous described
			sRGB_companding_LUT(bit_depth, tl_color_get_green_output_LUT(color_processor_handle));
			sRGB_companding_LUT(bit_depth, tl_color_get_blue_output_LUT(color_processor_handle));
			tl_color_enable_output_LUTs(color_processor_handle, 1, 1, 1);	// applied preset LUT to data
		}
		else {			// otherwise create RGB image directly from monochrome image using built-in api
			if (tl_mono_to_color_processing_initialize()) { std::cout << "failed to initialize mono to color processing" << std::endl; return 1; }	// initialize mono to color dll and sdk
			is_mono_to_color_sdk_open = 1;

			// color filter type and color filter array phase are needed: Thorlabs cameras have a color filter in front of the sensor
			tl_mono_to_color_create_mono_to_color_processor(camera_sensor_type, color_filter_array_phase, color_correction_matrix, default_white_balance_matrix, bit_depth, &mono_to_color_processor_handle);	// construct a mono to color processor
			if (!mono_to_color_processor_handle) { std::cout << "failed to contruct a mono to color processor" << std::endl; return 1; }

			// standard RGB (sRGB) performs nonlinear transformation to original RGB intensities, resembling how human eyes receive them
			// linear sRGB performs linear transformation to original RGB intensities, which reflects more of the original intensities or color
			// to use linear sRGB when the actual intensities of the raw image data are important, and sRGB when the image needs to look accurate to the human eyes
			tl_mono_to_color_set_color_space(mono_to_color_processor_handle, TL_MONO_TO_COLOR_SPACE_SRGB);	// set color space to sRGB (nonlinear)
			tl_mono_to_color_set_output_format(mono_to_color_processor_handle, TL_COLOR_FORMAT_RGB_PIXEL);	// set output color format to RGB
			tl_mono_to_color_set_red_gain(mono_to_color_processor_handle, default_white_balance_matrix[0]);	// use default white balance for gain
			tl_mono_to_color_set_green_gain(mono_to_color_processor_handle, default_white_balance_matrix[4]);
			tl_mono_to_color_set_blue_gain(mono_to_color_processor_handle, default_white_balance_matrix[8]);
		}

		if (tl_camera_set_frames_per_trigger_zero_for_unlimited(camera_handle, 0)) { std::cout << "failed to set trigger frame count" << std::endl; return 1; }	// continuous buffering when set to 0
		if (d_thread) {
			if (tl_camera_set_frame_available_callback(camera_handle, frame_available_callback, 0)) { std::cout << "failed to set the frame available callback" << std::endl; return 1; }	// register a callback function to receive a notification when an image is available
		}
		else {
			if (tl_camera_set_image_poll_timeout(camera_handle, 1000)) { std::cout << "failed to set the  frame available polling" << std::endl; return 1; }	// 1000 = 1s wait for a frame to arrive during a poll
		}

		return 0;
	}

	void thorcam::disconnect() {
		if (camera_handle) {
			if (tl_camera_close_camera(camera_handle)) { std::cout << "failed to close camera" << std::endl; }
			camera_handle = 0;
		}
		if (is_camera_sdk_open) {
			if (tl_camera_close_sdk()) { std::cout << "failed to close sdk" << std::endl; }
			is_camera_sdk_open = 0;
		}
		if (is_camera_dll_open) {
			if (tl_camera_sdk_dll_terminate()) { std::cout << "failed to close dll" << std::endl; }
			is_camera_dll_open = 0;
		}
		if (d_demosaic) {
			if (color_processor_handle) { if (tl_color_destroy_color_processor(color_processor_handle)) { std::cout << "failed to destroy color processor" << std::endl; } }
			if (is_color_processing_sdk_open) { if (tl_color_processing_terminate()) { std::cout << "failed to close color processing" << std::endl; } }
			if (is_demosaic_sdk_open) { if (tl_demosaic_terminate()) { std::cout << "failed to close demosaic sdk" << std::endl; } }
		}
		else {
			if (mono_to_color_processor_handle) {
				if (tl_mono_to_color_destroy_mono_to_color_processor(mono_to_color_processor_handle)) { std::cout << "failed to destroy mono to color processor" << std::endl; }
				mono_to_color_processor_handle = 0;
			}
			if (is_mono_to_color_sdk_open) {
				if (tl_mono_to_color_processing_terminate()) { std::cout << "failed to close mono to color sdk" << std::endl; }
				is_mono_to_color_sdk_open = 0;
			}
		}
		if (d_thread) {
			if (frame_acquired_event) { if (!CloseHandle(frame_acquired_event)) { std::cout << "failed to close concurrent data structure" << std::endl; } }
		}
		if (callback_image_buffer_copy) {
			delete[] callback_image_buffer_copy;
			callback_image_buffer_copy = 0;
		}
		if (poll_image_buffer_copy) {
			delete[] poll_image_buffer_copy;
			poll_image_buffer_copy = 0;
		}
		if (output_buffer) {
			delete[] output_buffer;
			output_buffer = 0;
		}
		if (demosaic_buffer) {
			delete[] demosaic_buffer;
			demosaic_buffer = 0;
		}
		if (output_buffer_24) {
			delete[] output_buffer_24;
			output_buffer_24 = 0;
		}
	}

	void thorcam::fire() {	
		tl_camera_arm(camera_handle, 1);	// arm camera and set the number of frames to allocate in the internal image buffer to 2
		
		if (d_thread)
			if (is_first_frame_finished)
				is_first_frame_finished = 0;	// now start frame buffer

		tl_camera_issue_software_trigger(camera_handle);	// sending a trigger command to the camera via USB 3.0
		if (d_thread) {	// multi thread
			for (;;) {	// wait for getting one image from buffer
				WaitForSingleObject(frame_acquired_event, INFINITE);
				if (is_first_frame_finished) break;
			}
		}
		else {			// single thread
			unsigned short *image_buffer = 0;
			int frame_count = 0;
			unsigned char *metadata = 0;
			int metadata_size_in_bytes = 0;

			while (!image_buffer) {	// poll for one image
				tl_camera_get_pending_frame_or_null(camera_handle, &image_buffer, &frame_count, &metadata, &metadata_size_in_bytes);
			}
			memcpy(poll_image_buffer_copy, image_buffer, (sizeof(unsigned short) * width * height));
		}
		//std::cout << "image #" << countI << " received..." << std::endl;	// now callback_image_buffer_copy has the unprocessed image

		if (d_demosaic) {
			// demosaic monochrome image data and create RGB data, expanding a single channel monochrome pixel data into three color channels of pixel data
			if (d_thread)
				tl_demosaic_transform_16_to_48(width, height, 0, 0, color_filter_array_phase, TL_COLOR_FORMAT_RGB_PIXEL, TL_COLOR_FILTER_TYPE_BAYER, bit_depth, callback_image_buffer_copy, demosaic_buffer);
			else
				tl_demosaic_transform_16_to_48(width, height, 0, 0, color_filter_array_phase, TL_COLOR_FORMAT_RGB_PIXEL, TL_COLOR_FILTER_TYPE_BAYER, bit_depth, poll_image_buffer_copy, demosaic_buffer);
			if (d_compression)
				tl_color_transform_48_to_24(color_processor_handle
					, demosaic_buffer                   // input buffer
					, TL_COLOR_FORMAT_RGB_PIXEL         // input buffer format, could be BGR or RGB
					, 0                                 // blue minimum clamp value
					, ((1 << bit_depth) - 1)            // blue maximum clamp value 
					, 0                                 // green minimum clamp value
					, ((1 << bit_depth) - 1)            // green maximum clamp value
					, 0                                 // red minimum clamp value
					, ((1 << bit_depth) - 1)            // red maximum clamp value
					, 8 - bit_depth                     // blue shift distance (negative: bit-shift data right, positive: bit-shift data left) 
					, 8 - bit_depth                     // green shift distance (negative: bit-shift data right, positive: bit-shift data left) 
					, 8 - bit_depth					    // red shift distance (negative: bit-shift data right, positive: bit-shift data left) 
					, output_buffer_24                  // output buffer
					, TL_COLOR_FORMAT_RGB_PIXEL         // output buffer format
					, width * height);					// number of pixels in the image
			else
				tl_color_transform_48_to_48(color_processor_handle
					, demosaic_buffer                   // input buffer
					, TL_COLOR_FORMAT_RGB_PIXEL         // input buffer format, could be BGR or RGB
					, 0                                 // blue minimum clamp value
					, ((1 << bit_depth) - 1)            // blue maximum clamp value 
					, 0                                 // green minimum clamp value
					, ((1 << bit_depth) - 1)            // green maximum clamp value
					, 0                                 // red minimum clamp value
					, ((1 << bit_depth) - 1)            // red maximum clamp value
					, 0                                 // blue shift distance (negative: bit-shift data right, positive: bit-shift data left) 
					, 0                                 // green shift distance (negative: bit-shift data right, positive: bit-shift data left) 
					, 0                                 // red shift distance (negative: bit-shift data right, positive: bit-shift data left) 
					, output_buffer                     // output buffer
					, TL_COLOR_FORMAT_RGB_PIXEL         // output buffer format
					, width * height);					// number of pixels in the image
		}
		else {
			if (d_thread) {	// using callbacks
				if (d_compression)
					tl_mono_to_color_transform_to_24(mono_to_color_processor_handle, callback_image_buffer_copy, width, height, output_buffer_24);
				else
					tl_mono_to_color_transform_to_48(mono_to_color_processor_handle, callback_image_buffer_copy, width, height, output_buffer);
			}
			else {			// using polls
				if (d_compression)
					tl_mono_to_color_transform_to_24(mono_to_color_processor_handle, poll_image_buffer_copy, width, height, output_buffer_24);
				else
					tl_mono_to_color_transform_to_48(mono_to_color_processor_handle, poll_image_buffer_copy, width, height, output_buffer);
			}
		}
		if (tl_camera_disarm(camera_handle)) { std::cout << "failed to disarm camera" << std::endl; }	// disarm camera
	}

	void thorcam::save(int count, std::string suffix) {
		std::string dir = output_dir + suffix;
		_mkdir(dir.c_str());	// create a folder if not exist
		std::stringstream ss;
		std::stringstream n;
		n << std::setfill('0') << std::setw(3) << count;
		ss << dir << "/" << n.str() << "." << format;
		std::string image_name = ss.str();
		if (d_compression) {	// save in 24bpp
			stim::image<unsigned char> I(&output_buffer_24[0], width, height, 3);
			I.save(image_name);
		}
		else {				// save in 48bpp, could do 32bpp too with unsigned short * 4 if needed
			stim::image<unsigned short> I(&output_buffer[0], width, height, 3);
			I.save(image_name);
		}
	}
}
