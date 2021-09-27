#include "fmeasure.h"

namespace fm {
	enum fmetric {GLSD, SPFQ, BREN, HISE};
	
	// convert from rgb to grayscale, by default using CCIR 601
	// luma 1->CCIR 601, 2->BT.709, 3->SMPTE 240M
	template<typename T>
	inline void rgb2gray(const T *in, T *out, int width, int height, int luma) {
		//std::vector<unsigned char> R;	// R-channel
		//std::vector<unsigned char> G;	// G-channel
		//std::vector<unsigned char> B;	// B-channel
		//for (int i = 0; i < num; i++) {
		//	R.push_back(*(in + i * 3 + 0));
		//	G.push_back(*(in + i * 3 + 1));
		//	B.push_back(*(in + i * 3 + 2));
		//}
		//T *R = (T*)malloc(pnum * sizeof(T));	// R-channel
		//T *G = (T*)malloc(pnum * sizeof(T));	// G-channel
		//T *B = (T*)malloc(pnum * sizeof(T));	// B-channel
		//for (int i = 0; i < pnum; i++) {	// split rgb channels
		//	*(R + i) = *(in + i * 3 + 0);
		//	*(G + i) = *(in + i * 3 + 1);
		//	*(B + i) = *(in + i * 3 + 2);
		//	out[i] = 0.2989*R[i] + 0.5870*G[i] + 0.1140*B[i];
		//}
		//// do whatever you want with individual channels
		//delete[] R;	// free memory
		//delete[] G;
		//delete[] B;
		int pnum = width * height;
		for (int i = 0; i < pnum; i++) {
			if (luma == 1)
				out[i] = 0.2990 * in[i * 3 + 0] + 0.5870 * in[i * 3 + 1] + 0.1140 * in[i * 3 + 2];	// CCIR 601 luma criterio
			else if (luma == 2)
				out[i] = 0.2126 * in[i * 3 + 0] + 0.7152 * in[i * 3 + 1] + 0.0722 * in[i * 3 + 2];	// BT. 709 luma criterio
			else if (luma == 3)
				out[i] = 0.2120 * in[i * 3 + 0] + 0.7010 * in[i * 3 + 1] + 0.0870 * in[i * 3 + 2];	// SMPTE 240M luma criterio
		}		
	}

	// focus measure #1 -- graylevel standard deviation / graylevel variance
	template<typename T>
	float glsd(const T *in, int width, int height) {
		float result = 0.0f;
		int pnum = width * height;
		int sum = 0;
		for (int i = 0; i < pnum; i++)
			sum += in[i];
		float mean = (float)sum / pnum;
		for (int i = 0; i < pnum; i++)
			result += std::pow((in[i] - mean), 2);
		result = std::sqrt(result / pnum);

		return result;
	}

	// focus measure #2 -- spatial frequency
	template<typename T>
	float spfq(const T *in, int width, int height) {
		float result = 0.0f;
		int pnum = width * height;
		int hsum = 0; int vsum = 0;
		for (int i = 0; i < height; i++)
			for (int j = 1; j < width; j++)
				hsum += (int)std::pow((in[i * width + j] - in[i * width + (j - 1)]), 2);	// horizontal spatial gradient
		for (int i = 0; i < width; i++)
			for (int j = 1; j < height; j++)
				vsum += (int)std::pow((in[j * width + i] - in[(j - 1) * width + i]), 2);	// vertical spatial gradient
		float hmean = (float)hsum / pnum;
		float vmean = (float)vsum / pnum;
		result = std::sqrt(hmean + vmean);

		return result;
	}

	// focus measure #3 -- Brenner's first differentiation
	template<typename T>
	float bren(const T *in, int width, int height) {
		float result = 0.0f;
		int pnum = width * height;
		int hsum = 0; int vsum = 0;
		for (int i = 0; i < height; i++)
			for (int j = 2; j < width; j++)
				hsum += (int)std::pow((in[i * width + j] - in[i * width + (j - 2)]), 2);	// horizontal first derivative
		for (int i = 0; i < width; i++)
			for (int j = 2; j < height; j++)
				vsum += (int)std::pow((in[j * width + i] - in[(j - 2) * width + i]), 2);	// vertical first derivative
		float hmean = (float)hsum / pnum;
		float vmean = (float)vsum / pnum;
		result = std::max(hmean, vmean);
		result = std::sqrt(result);

		return result;
	}

	// focus measure #4 -- histogram entropy
	template<typename T>
	float hise(const T *in, int width, int height) {
		float result = 0.0f;
		int pnum = width * height;
		std::vector<int> value;
		std::vector<float> count;
		for (int i = 0; i < pnum; i++) {
			auto iter = std::find(value.begin(), value.end(), in[i]);
			if (iter == value.end()) {
				value.push_back(in[i]);
				count.push_back(1.0f);
			}
			else {
				int index = (int)std::distance(value.begin(), iter);
				count[index]++;
			}
		}
		for (size_t i = 0; i < count.size(); i++) {
			count[i] = count[i] / pnum;
			result += -count[i] * std::log2f(count[i]);
		}

		return result;
	}
	
	// compute global focus measure
	template<typename T>
	float eval_fm(const T *in, int width, int height, fmetric alg) {
		float result = 0.0f;
		T *gimg = (T*)malloc(width * height * sizeof(T));
		rgb2gray<T>(in, gimg, width, height);
		switch (alg) {	// choose focus measure algorithm to evaluate in-focus/out-of-focus
		case GLSD:
			result = glsd<T>(gimg, width, height);
			break;
		case SPFQ:
			result = spfq<T>(gimg, width, height);
			break;
		case BREN:
			result = bren<T>(gimg, width, height);
			break;
		case HISE:
			result = hise<T>(gimg, width, height);
			break;
		}
		
		return result;	
	}
}

// do all forward declaration for all template function to avoid LINK errors
template void fm::rgb2gray<unsigned char>(const unsigned char *in, unsigned char *out, int width, int height);
template void fm::rgb2gray<unsigned short>(const unsigned short *in, unsigned short *out, int width, int height);
template float fm::glsd<unsigned char>(const unsigned char *in, int width, int height);
template float fm::glsd<unsigned short>(const unsigned short *in, int width, int height);
template float fm::spfq<unsigned char>(const unsigned char *in, int width, int height);
template float fm::spfq<unsigned short>(const unsigned short *in, int width, int height);
template float fm::bren<unsigned char>(const unsigned char *in, int width, int height);
template float fm::bren<unsigned short>(const unsigned short *in, int width, int height);
template float fm::hise<unsigned char>(const unsigned char *in, int width, int height);
template float fm::hise<unsigned short>(const unsigned short *in, int width, int height);
template float fm::eval_fm<unsigned char>(const unsigned char *in, int width, int height, fmetric alg);
template float fm::eval_fm<unsigned short>(const unsigned short *in, int width, int height, fmetric alg);